/*
  MCP2515.cpp - Library for Microchip MCP2515 CAN Controller
  
  Author: David Harding
  Maintainer: RechargeCar Inc (http://rechargecar.com)
  Further Modification: Collin Kidder
  
  Created: 11/08/2010
  
  For further information see:
  
  http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
  http://en.wikipedia.org/wiki/CAN_bus


  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"
#include "SPI.h"
#include "MCP2515.h"
#include "MCP2515_defs.h"

SPISettings canSPISettings(1000000, MSBFIRST, SPI_MODE0);

MCP2515::MCP2515(uint8_t CS_Pin, uint8_t INT_Pin) {
  pinMode(CS_Pin, OUTPUT);
  digitalWrite(CS_Pin,HIGH);
  pinMode(INT_Pin,INPUT);
  digitalWrite(INT_Pin,HIGH);
  
  _CS = CS_Pin;
  _INT = INT_Pin;
  
  savedBaud = 0;
  savedFreq = 0;
  running = 0; 
  InitBuffers();
}

//set all buffer counters to zero to reset them
void MCP2515::InitBuffers() {
  rx_frame_read_pos = 0;
  rx_frame_write_pos = 0;
  tx_frame_read_pos = 0;
  tx_frame_write_pos = 0;
}  

/*
  Initialize MCP2515
  
  int CAN_Bus_Speed = transfer speed in kbps (or raw CAN speed in bits per second)
  int Freq = MCP2515 oscillator frequency in MHz
  int SJW = Synchronization Jump Width Length bits - 1 to 4 (see data sheet)
  
  returns baud rate set
  
  Sending a bus speed of 0 kbps initiates AutoBaud and returns zero if no
  baud rate could be determined.  There must be two other active nodes on the bus!
*/
int MCP2515::Init(uint32_t CAN_Bus_Speed, uint8_t Freq) {
  if(CAN_Bus_Speed>0) {
    if(_init(CAN_Bus_Speed, Freq, 1, false)) {
		savedBaud = CAN_Bus_Speed;
		savedFreq = Freq;
		running = 1;
	    return CAN_Bus_Speed;
    }
  } else {
	int i=0;
	uint8_t interruptFlags = 0;
	for(i=5; i<1000; i=i+5) {
	  if(_init(i, Freq, 1, true)) {
		// check for bus activity
		Write(CANINTF,0);
		delay(500); // need the bus to be communicating within this time frame
		if(Interrupt()) {
		  // determine which interrupt flags have been set
		  interruptFlags = Read(CANINTF);
		  if(!(interruptFlags & MERRF)) {
		    // to get here we must have received something without errors
		    Mode(MODE_NORMAL);
			savedBaud = i;
			savedFreq = Freq;	
			running = 1;
			return i;
		  }
		}
	  }
	}
  }
  return 0;
}

int MCP2515::Init(uint32_t CAN_Bus_Speed, uint8_t Freq, uint8_t SJW) {
  if(SJW < 1) SJW = 1;
  if(SJW > 4) SJW = 4;
  if(CAN_Bus_Speed>0) {
    if(_init(CAN_Bus_Speed, Freq, SJW, false)) {
		savedBaud = CAN_Bus_Speed;
		savedFreq = Freq;
		running = 1;
	    return CAN_Bus_Speed;
    }
  } else {
	int i=0;
	uint8_t interruptFlags = 0;
	for(i=5; i<1000; i=i+5) {
	  if(_init(i, Freq, SJW, true)) {
		// check for bus activity
		Write(CANINTF,0);
		delay(500); // need the bus to be communicating within this time frame
		if(Interrupt()) {
		  // determine which interrupt flags have been set
		  interruptFlags = Read(CANINTF);
		  if(!(interruptFlags & MERRF)) {
		    // to get here we must have received something without errors
		    Mode(MODE_NORMAL);
			savedBaud = i;
			savedFreq = Freq;
			running = 1;
			return i;
		  }
		}
	  }
	}
  }
  return 0;
}

bool MCP2515::_init(uint32_t CAN_Bus_Speed, uint8_t Freq, uint8_t SJW, bool autoBaud) {
  
  // Reset MCP2515 which puts it in configuration mode
  Reset();

  Write(CANINTE,0); //disable all interrupts during init
  Write(TXB0CTRL, 0); //reset transmit control
  Write(TXB1CTRL, 0); //reset transmit control
  Write(TXB2CTRL, 0); //reset transmit control
  
  // Calculate bit timing registers
  uint8_t BRP;
  float TQ;
  uint8_t BT;
  float tempBT;
  float freqMhz = Freq * 1000000.0;
  float bestMatchf = 10.0;
  int bestMatchIdx = 10;
  float savedBT;

  float speed = CAN_Bus_Speed;
  if (speed > 5000.0) speed *= 0.001;

  float NBT = 1.0 / (speed * 1000.0); // Nominal Bit Time - How long a single CAN bit should take

  //Now try each divisor to see which can most closely match the target.
  for(BRP=0; BRP < 63; BRP++) {
    TQ = 2.0 * (float)(BRP + 1) / (float)freqMhz;
    tempBT = NBT / TQ;
#ifdef DEBUG_SETUP
    SerialUSB.print("BRP: ");
    SerialUSB.print(BRP);
    SerialUSB.print("  tempBT: ");
    SerialUSB.println(tempBT);
#endif
    BT = (int)tempBT;
    if ( (tempBT - BT) < bestMatchf)
    {
        if (BT > 7 && BT < 25)
        {
            bestMatchf = (tempBT - BT);
            bestMatchIdx = BRP;
            savedBT = BT;
        }
    }
  }

  BT = savedBT;
  BRP = bestMatchIdx;
#ifdef DEBUG_SETUP  
  SerialUSB.print("BRP: ");
  SerialUSB.print(BRP);
  SerialUSB.print("  BT: ");
  SerialUSB.println(BT);
#endif
  
  byte SPT = (0.7 * BT); // Sample point
  byte PRSEG = (SPT - 1) / 2;
  byte PHSEG1 = SPT - PRSEG - 1;
  byte PHSEG2 = BT - PHSEG1 - PRSEG - 1;
#ifdef DEBUG_SETUP
  SerialUSB.print("PROP: ");
  SerialUSB.print(PRSEG);
  SerialUSB.print("  SEG1: ");
  SerialUSB.print(PHSEG1);
  SerialUSB.print("  SEG2: ");
  SerialUSB.println(PHSEG2);
#endif
  // Programming requirements
  if(PRSEG + PHSEG1 < PHSEG2) 
  {
      SerialUSB.println("PRSEG + PHSEG1 less than PHSEG2!");
      return false;
  }
  if(PHSEG2 <= SJW) 
  {
      SerialUSB.println("PHSEG2 less than SJW");
      return false;
  }
  
  uint8_t BTLMODE = 1;
  uint8_t SAMPLE = 0;
  
  // Set registers
  byte data = (((SJW-1) << 6) | BRP);
  Write(CNF1, data);
  Write(CNF2, ((BTLMODE << 7) | (SAMPLE << 6) | ((PHSEG1-1) << 3) | (PRSEG-1)));
  Write(CNF3, (B10000000 | (PHSEG2-1)));
  Write(TXRTSCTRL,0);
  
  if(!autoBaud) {
    // Return to Normal mode
    if(!Mode(MODE_NORMAL)) 
    {
        SerialUSB.println("Could not enter normal mode");
        return false;
    }
  } else {
    // Set to Listen Only mode
    if(!Mode(MODE_LISTEN)) 
    {
        SerialUSB.println("Could not enter listen only mode");
        return false;
    }
  }
  // Enable all interupts
  Write(CANINTE,255);
  
  // Test that we can read back from the MCP2515 what we wrote to it
  byte rtn = Read(CNF1);
  if (rtn == data) return true;
  else 
  {
    SerialUSB.println(data, HEX);
    SerialUSB.println(rtn, HEX);
    return false;
  }

  return false;
}

void MCP2515::Reset() {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_RESET);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

uint8_t MCP2515::Read(uint8_t address) {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_READ);
  SPI.transfer(address);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
  return data;
}

void MCP2515::Read(uint8_t address, uint8_t data[], uint8_t bytes) {
  // allows for sequential reading of registers starting at address - see data sheet
  uint8_t i;
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_READ);
  SPI.transfer(address);
  for(i=0;i<bytes;i++) {
    data[i] = SPI.transfer(0x00);
  }
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

Frame MCP2515::ReadBuffer(uint8_t buffer) {
 
  // Reads an entire RX buffer.
  // buffer should be either RXB0 or RXB1
  
  Frame message;
  
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_READ_BUFFER | (buffer<<1));
  uint8_t byte1 = SPI.transfer(0x00); // RXBnSIDH
  uint8_t byte2 = SPI.transfer(0x00); // RXBnSIDL
  uint8_t byte3 = SPI.transfer(0x00); // RXBnEID8
  uint8_t byte4 = SPI.transfer(0x00); // RXBnEID0
  uint8_t byte5 = SPI.transfer(0x00); // RXBnDLC

  message.extended = (byte2 & B00001000);

  if(message.extended) {
    message.id = (byte1>>3);
    message.id = (message.id<<8) | ((byte1<<5) | ((byte2>>5)<<2) | (byte2 & B00000011));
    message.id = (message.id<<8) | byte3;
    message.id = (message.id<<8) | byte4;
  } else {
    message.id = ((byte1>>5)<<8) | ((byte1<<3) | (byte2>>5));
  }

  message.rtr=(byte5 & B01000000);
  message.length = (byte5 & B00001111);  // Number of data bytes
  for(int i=0; i<message.length; i++) {
    message.data.byte[i] = SPI.transfer(0x00);
  }
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();

  return message;
}

void MCP2515::Write(uint8_t address, uint8_t data) {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_WRITE);
  SPI.transfer(address);
  SPI.transfer(data);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

void MCP2515::Write(uint8_t address, uint8_t data[], uint8_t bytes) {
  // allows for sequential writing of registers starting at address - see data sheet
  uint8_t i;
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_WRITE);
  SPI.transfer(address);
  for(i=0;i<bytes;i++) {
    SPI.transfer(data[i]);
  }
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

void MCP2515::SendBuffer(uint8_t buffers) {
  // buffers should be any combination of TXB0, TXB1, TXB2 ORed together, or TXB_ALL
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_RTS | buffers);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

void MCP2515::LoadBuffer(uint8_t buffer, Frame *message) {
 
  // buffer should be one of TXB0, TXB1 or TXB2
  if(buffer==TXB0) buffer = 0; //the values we need are 0, 2, 4 TXB1 and TXB2 are already 2 / 4

  uint8_t byte1=0; // TXBnSIDH
  uint8_t byte2=0; // TXBnSIDL
  uint8_t byte3=0; // TXBnEID8
  uint8_t byte4=0; // TXBnEID0
  uint8_t byte5=0; // TXBnDLC

  if(message->extended) {
    byte1 = byte((message->id<<3)>>24); // 8 MSBits of SID
	byte2 = byte((message->id<<11)>>24) & B11100000; // 3 LSBits of SID
	byte2 = byte2 | byte((message->id<<14)>>30); // 2 MSBits of EID
	byte2 = byte2 | B00001000; // EXIDE
    byte3 = byte((message->id<<16)>>24); // EID Bits 15-8
    byte4 = byte((message->id<<24)>>24); // EID Bits 7-0
  } else {
    byte1 = byte((message->id<<21)>>24); // 8 MSBits of SID
	byte2 = byte((message->id<<29)>>24) & B11100000; // 3 LSBits of SID
    byte3 = 0; // TXBnEID8
    byte4 = 0; // TXBnEID0
  }
  byte5 = message->length;
  if(message->rtr) {
    byte5 = byte5 | B01000000;
  }
  
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_LOAD_BUFFER | buffer);  
  SPI.transfer(byte1);
  SPI.transfer(byte2);
  SPI.transfer(byte3);
  SPI.transfer(byte4);
  SPI.transfer(byte5);
 
  for(int i=0;i<message->length;i++) {
    SPI.transfer(message->data.byte[i]);
  }
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

uint8_t MCP2515::Status() {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_STATUS);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
  return data;
  /*
  bit 7 - CANINTF.TX2IF
  bit 6 - TXB2CNTRL.TXREQ
  bit 5 - CANINTF.TX1IF
  bit 4 - TXB1CNTRL.TXREQ
  bit 3 - CANINTF.TX0IF
  bit 2 - TXB0CNTRL.TXREQ
  bit 1 - CANINTFL.RX1IF
  bit 0 - CANINTF.RX0IF
  */
}

uint8_t MCP2515::RXStatus() {
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_RX_STATUS);
  uint8_t data = SPI.transfer(0x00);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
  return data;
  /*
  bit 7 - CANINTF.RX1IF
  bit 6 - CANINTF.RX0IF
  bit 5 - 
  bit 4 - RXBnSIDL.EIDE
  bit 3 - RXBnDLC.RTR
  bit 2 | 1 | 0 | Filter Match
  ------|---|---|-------------
      0 | 0 | 0 | RXF0
	  0 | 0 | 1 | RXF1
	  0 | 1 | 0 | RXF2
	  0 | 1 | 1 | RXF3
	  1 | 0 | 0 | RXF4
	  1 | 0 | 1 | RXF5
	  1 | 1 | 0 | RXF0 (rollover to RXB1)
	  1 | 1 | 1 | RXF1 (rollover to RXB1)
  */
}

void MCP2515::BitModify(uint8_t address, uint8_t mask, uint8_t data) {
  // see data sheet for explanation
  SPI.beginTransaction(canSPISettings);
  digitalWrite(_CS,LOW);
  SPI.transfer(CAN_BIT_MODIFY);
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(_CS,HIGH);
  SPI.endTransaction();
}

bool MCP2515::Interrupt() {
  return (digitalRead(_INT)==LOW);
}

bool MCP2515::Mode(byte mode) {
  /*
  mode can be one of the following:
  MODE_CONFIG
  MODE_LISTEN
  MODE_LOOPBACK
  MODE_SLEEP
  MODE_NORMAL
  */
  BitModify(CANCTRL, B11100000, mode);
  delay(10); // allow for any transmissions to complete
  uint8_t data = Read(CANSTAT); // check mode has been set
  return ((data & mode)==mode);
}

/*Initializes all filters to either accept all frames or accept none
//This doesn't need to be called if you want to accept everything
//because that's the default. So, call this with permissive = false
//to state with an accept nothing state and then add acceptance masks/filters
//thereafter 
*/
void MCP2515::InitFilters(bool permissive) {
	long value;
	if (permissive) {
		value = 0;
	}	
	else {
		value = 0x7FF; //all 11 bits set
	}
	SetRXMask(MASK0, value, 0);
	SetRXMask(MASK1, value, 0);
	SetRXFilter(FILTER0, value, 0);
	SetRXFilter(FILTER1, value, 0);
	SetRXFilter(FILTER2, value, 0);
	SetRXFilter(FILTER3, value, 0);
	SetRXFilter(FILTER4, value, 0);
	SetRXFilter(FILTER5, value, 0);
}

/*
mask = either MASK0 or MASK1
MaskValue is either an 11 or 29 bit mask value to set
ext is true if the mask is supposed to be extended (29 bit)
*/
void MCP2515::SetRXMask(uint8_t mask, long MaskValue, bool ext) {
	uint8_t temp_buff[4];
	uint8_t oldMode;
	
	oldMode = Read(CANSTAT);
	Mode(MODE_CONFIG); //have to be in config mode to change mask
	
	if (ext) { //fill out all 29 bits
		temp_buff[0] = byte((MaskValue << 3) >> 24);
		temp_buff[1] = byte((MaskValue << 11) >> 24) & B11100000;
		temp_buff[1] |= byte((MaskValue << 14) >> 30);
		temp_buff[2] = byte((MaskValue << 16)>>24);
		temp_buff[3] = byte((MaskValue << 24)>>24);
	}
	else { //make sure to set mask as 11 bit standard mask
		temp_buff[0] = byte((MaskValue << 21)>>24);
		temp_buff[1] = byte((MaskValue << 29) >> 24) & B11100000;
		temp_buff[2] = 0;
		temp_buff[3] = 0;
	}
	
	Write(mask, temp_buff, 4); //send the four byte mask out to the proper address
	
	Mode(oldMode);
}

/*
filter = FILTER0, FILTER1, FILTER2, FILTER3, FILTER4, FILTER5 (pick one)
FilterValue = 11 or 29 bit filter to use
ext is true if this filter should apply to extended frames or false if it should apply to standard frames.
Do note that, while this function looks a lot like the mask setting function is is NOT identical
It might be able to be though... The setting of EXIDE would probably just be ignored by the mask
*/
void MCP2515::SetRXFilter(uint8_t filter, long FilterValue, bool ext) {
	uint8_t temp_buff[4];
	uint8_t oldMode;
		
	oldMode = Read(CANSTAT);

	Mode(MODE_CONFIG); //have to be in config mode to change mask
	
	if (ext) { //fill out all 29 bits
		temp_buff[0] = byte((FilterValue << 3) >> 24);
		temp_buff[1] = byte((FilterValue << 11) >> 24) & B11100000;
		temp_buff[1] |= byte((FilterValue << 14) >> 30);
		temp_buff[1] |= B00001000; //set EXIDE
		temp_buff[2] = byte((FilterValue << 16)>>24);
		temp_buff[3] = byte((FilterValue << 24)>>24);
	}
	else { //make sure to set mask as 11 bit standard mask
		temp_buff[0] = byte((FilterValue << 21)>>24);
		temp_buff[1] = byte((FilterValue << 29) >> 24) & B11100000;
		temp_buff[2] = 0;
		temp_buff[3] = 0;
	}
	
	Write(filter, temp_buff, 4); //send the four byte mask out to the proper address
	
	Mode(oldMode);
}

//Places the given frame into the receive queue
void MCP2515::EnqueueRX(Frame& newFrame) {
	uint8_t counter;
	rx_frames[rx_frame_write_pos].id = newFrame.id;
	rx_frames[rx_frame_write_pos].rtr = newFrame.rtr;
	rx_frames[rx_frame_write_pos].extended = newFrame.extended;
	rx_frames[rx_frame_write_pos].length = newFrame.length;
	for (counter = 0; counter < 8; counter++) rx_frames[rx_frame_write_pos].data.byte[counter] = newFrame.data.byte[counter];
	rx_frame_write_pos = (rx_frame_write_pos + 1) % 8;
}

//Places the given frame into the transmit queue
//Well, maybe. If there is currently an open hardware buffer
//it will place it into hardware immediately instead of using
//the software queue
void MCP2515::EnqueueTX(Frame& newFrame) {
	uint8_t counter;
	uint8_t status = Status() & 0b01010100; //mask for only the transmit buffer empty bits
	
	//don't allow sending or queueing of frames if we're not properly initialized
	if (running == 0) {
		return;
	}
		
	if (status != 0b01010100) { //found an open slot
		if ((status & 0b00000100) == 0) { //transmit buffer 0 is open
			LoadBuffer(TXB0, &newFrame);
			SendBuffer(TXB0);
		}
		else if ((status & 0b00010000) == 0) { //transmit buffer 1 is open
			LoadBuffer(TXB1, &newFrame);
			SendBuffer(TXB1);
		}
		else { // must have been buffer 2 then.
			LoadBuffer(TXB2, &newFrame);
			SendBuffer(TXB2);
		}
	}
	else { //hardware is busy. queue it in software
		if (tx_frame_write_pos != tx_frame_read_pos) { //don't add another frame if the buffer is already full
			tx_frames[tx_frame_write_pos].id = newFrame.id;
			tx_frames[tx_frame_write_pos].rtr = newFrame.rtr;
			tx_frames[tx_frame_write_pos].extended = newFrame.extended;
			tx_frames[tx_frame_write_pos].length = newFrame.length;
			for (counter = 0; counter < 8; counter++) tx_frames[tx_frame_write_pos].data.byte[counter] = newFrame.data.byte[counter];
			tx_frame_write_pos = (tx_frame_write_pos + 1) % 8;
		}		
	}		
}

bool MCP2515::GetRXFrame(Frame &frame) {
	uint8_t counter;
	if (rx_frame_read_pos != rx_frame_write_pos) {
		frame.id = rx_frames[rx_frame_read_pos].id;
		frame.rtr = rx_frames[rx_frame_read_pos].rtr;
		frame.extended = rx_frames[rx_frame_read_pos].extended;
		frame.length = rx_frames[rx_frame_read_pos].length;
		for (counter = 0; counter < 8; counter++) frame.data.byte[counter] = rx_frames[rx_frame_read_pos].data.byte[counter];
		rx_frame_read_pos = (rx_frame_read_pos + 1) % 8;
		return true;
	}
	else return false;
}

void MCP2515::intHandler(void) {
    Frame message;
    // determine which interrupt flags have been set
    uint8_t interruptFlags = Read(CANINTF);
    //Now, acknowledge the interrupts by clearing the intf bits
    Write(CANINTF, 0); 	
    
    if(interruptFlags & RX0IF) {
      // read from RX buffer 0
		message = ReadBuffer(RXB0);
     	EnqueueRX(message);
    }
    if(interruptFlags & RX1IF) {
      // read from RX buffer 1
      message = ReadBuffer(RXB1);
      EnqueueRX(message);
    }
    if(interruptFlags & TX0IF) {
		// TX buffer 0 sent
       if (tx_frame_read_pos != tx_frame_write_pos) {
			LoadBuffer(TXB0, (Frame *)&tx_frames[tx_frame_read_pos]);
		   	SendBuffer(TXB0);
			tx_frame_read_pos = (tx_frame_read_pos + 1) % 8;
	   }
    }
    if(interruptFlags & TX1IF) {
		// TX buffer 1 sent
	  if (tx_frame_read_pos != tx_frame_write_pos) {
		  LoadBuffer(TXB1, (Frame *)&tx_frames[tx_frame_read_pos]);
		  SendBuffer(TXB1);
		  tx_frame_read_pos = (tx_frame_read_pos + 1) % 8;
	  }
    }
    if(interruptFlags & TX2IF) {
		// TX buffer 2 sent
		if (tx_frame_read_pos != tx_frame_write_pos) {
			LoadBuffer(TXB2, (Frame *)&tx_frames[tx_frame_read_pos]);
			SendBuffer(TXB2);
			tx_frame_read_pos = (tx_frame_read_pos + 1) % 8;
		}
    }
    if(interruptFlags & ERRIF) {
      if (running == 1) { //if there was an error and we had been initialized then try to fix it by reinitializing
		  running = 0;
		  InitBuffers();
		  Init(savedBaud, savedFreq);
	  }
    }
    if(interruptFlags & MERRF) {
      // error handling code
      // if TXBnCTRL.TXERR set then transmission error
      // if message is lost TXBnCTRL.MLOA will be set
      if (running == 1) { //if there was an error and we had been initialized then try to fix it by reinitializing
		running = 0;
		InitBuffers();
		Init(savedBaud, savedFreq);
	  }	  
    }
}

int MCP2515::watchFor()
{
	SetRXMask(MASK0, 0, true);
	SetRXMask(MASK1, 0, false);
	SetRXFilter(FILTER0, 0, true);
	SetRXFilter(FILTER2, 0, false);
	return 0;
}

int MCP2515::watchFor(uint32_t id)
{
	if (id > 0x7FF) SetRXFilter(id, 0x1FFFFFFF, true);
	else SetRXFilter(id, 0x7FF, false);
	return 0;
}

int MCP2515::watchFor(uint32_t id, uint32_t mask)
{
	if (id > 0x7FF) SetRXFilter(id, mask, true);
	else SetRXFilter(id, mask, false);
	return 0;
}

int MCP2515::watchForRange(uint32_t id1, uint32_t id2)
{
	uint32_t id = 0;
	uint32_t mask = 0;
	uint32_t temp;

	if (id1 > id2)
	{   //looks funny I know. In place swap with no temporary storage. Neato!
		id1 = id1 ^ id2;
		id2 = id1 ^ id2; //note difference here.
		id1 = id1 ^ id2;
	}

	id = id1;

	if (id2 <= 0x7FF) mask = 0x7FF;
	else mask = 0x1FFFFFFF;

	/* Here is a quick overview of the theory behind these calculations.
	   We start with mask set to 11 or 29 set bits (all 1's)
	   and id set to the lowest ID in the range.
	   From there we go through every single ID possible in the range. For each ID
	   we AND with the current ID. At the end only bits that never changed and were 1's
	   will still be 1's. This yields the ID we can match against to let these frames through
	   The mask is calculated by finding the bitfield difference between the lowest ID and
	   the current ID. This calculation will be 1 anywhere the bits were different. We invert
	   this so that it is 1 anywhere the bits where the same. Then we AND with the current Mask.
	   At the end the mask will be 1 anywhere the bits never changed. This is the perfect mask.
	*/
	for (int c = id1; c <= id2; c++)
	{
		id &= c;
		temp = (~(id1 ^ c)) & 0x1FFFFFFF;
		mask &= temp;
	}
	//output of the above crazy loop is actually the end result.
	if (id > 0x7FF) SetRXFilter(id, mask, true);
	else SetRXFilter(id, mask, false);
	return 0;

}
/*
void MCP2515::attachCANInterrupt(void (*cb)(CAN_FRAME *))
{
	cbCANFrame[6] = cb;
}

void MCP2515::attachCANInterrupt(uint8_t filter, void (*cb)(CAN_FRAME *))
{
	if ((filter < 0) || (filter > 5)) return;
	cbCANFrame[filter] = cb;
}

void MCP2515::detachCANInterrupt(uint8_t filter)
{
	if ((filter < 0) || (filter > 5)) return;
	cbCANFrame[filter] = 0;
}
*/
int MCP2515::available()
{
	int val;
	val = rx_frame_read_pos - rx_frame_write_pos;
	//Now, because this is a cyclic buffer it is possible that the ordering was reversed
	//So, handle that case
	if (val < 0) val += 8;
}
