/*
  MCP2515 CAN PID filtering
  
  Author: Andy Cook
  
  Created: 4/2013 by RechargeCar Inc.
  
  For further information see:
  
  http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
  http://en.wikipedia.org/wiki/CAN_bus
  http://en.wikipedia.org/wiki/OBD-II_PIDs

  The MCP2515 Library files also contain important information.
  
  This sketch is configured to work with the 'Macchina' Automotive Interface board 
  manufactured by RechargeCar Inc. CS_PIN and INT_PIN are specific to this board.
  
  This sketch shows a rough example for receiving OBD-specific messages over CAN. 
  It has not been thoroughly tested due to a lack of access to compatible vehicles
  by the developer.

  This example code is in the public domain.
  
*/

#include <SPI.h>
#include <MCP2515.h>

// Pin definitions specific to how the MCP2515 is wired up.
#define CS_PIN    85
#define RESET_PIN  7
#define INT_PIN    84

// Create CAN object with pins as defined
MCP2515 CAN(CS_PIN, RESET_PIN, INT_PIN);

byte intFlag;

void CANHandler() {
	CAN.intHandler();
        intFlag = 1;
}


Frame testMessage;
byte txLEDState;
unsigned long warningTimestamp;
unsigned int rpm;
unsigned char temp;
unsigned char mph;
unsigned char fuel;

int numdata;
boolean started=false;
char n[20];

void setup() {
        txLEDState = 0;
        warningTimestamp = 0;
        
        rpm = 0;
        temp = 0;
        mph = 0;
        fuel = 0;
        
	Serial.begin(115200);
	
	Serial.println("Initializing ...");

	// Set up SPI Communication
	// dataMode can be SPI_MODE0 or SPI_MODE3 only for MCP2515
	SPI.setClockDivider(SPI_CLOCK_DIV2);
	SPI.setDataMode(SPI_MODE0);
	SPI.setBitOrder(MSBFIRST);
	SPI.begin();
	
	// Initialize MCP2515 CAN controller at the specified speed and clock frequency
	// (Note:  This is the oscillator attached to the MCP2515, not the Arduino oscillator)
	//speed in KHz, clock in MHz

        // NOTE! This speed might need to change. Usually 250 or 500
	if(CAN.Init(250,16))
	{
		Serial.println("MCP2515 Init OK ...");
	} else {
		Serial.println("MCP2515 Init Failed ...");
	}
	
	attachInterrupt(6, CANHandler, FALLING);
	CAN.InitFilters(false);
	CAN.SetRXMask(MASK0, 0x7F8, 0); //match all but bottom four bits
        CAN.SetRXFilter(FILTER0, 0x7E8, 0); // Allows 0x7E8 through 0x7EF, 
                                      // to allow responses from up to 8 ECUs,
                                      // per the 2008 OBD requirements

	Serial.println("MCP2515 Ready ...");


      // The following code sends a message out on the bus requesting a list of PIDS that the vehicle
      // computer supports in the 0x01 to 0x20 range.
      testMessage.id = 0x07DF;
      testMessage.srr = 1;
      testMessage.rtr = 0;
      testMessage.ide = 0;
      testMessage.dlc = 3;
      testMessage.data[0] = 0x02;  // Number of data bytes to follow
      testMessage.data[1] = 0x01;  // Mode
      testMessage.data[2] = 0x00;  // Requested PID. In this case, "PIDs supported, 0x01-0x20"
  
    CAN.EnqueueTX(testMessage);
}

byte i=0;

// CAN message frame (actually just the parts that are exposed by the MCP2515 RX/TX buffers)
Frame message;

void loop() {
	if (CAN.GetRXFrame(message)) {
		// Print message
                // This prints every message, which can be overwhelming
                // if the CAN bus on the OBDII port has lots of other
                // chatter on it beyond OBDII things.
                // This can be useful, though, if you're trying to find
                // whether there's any
		Serial.print("ID: ");
		Serial.println(message.id,HEX);
		Serial.print("Extended: ");
		if(message.ide) {
			Serial.println("Yes");
		} else {
			Serial.println("No");
		}
		Serial.print("DLC: ");
		Serial.println(message.dlc,DEC);
		for(i=0;i<message.dlc;i++) {
			Serial.print(message.data[i],HEX);
			Serial.print(" ");
		}
		Serial.println();
		Serial.println();

                // This filters for just OBDII PID messages
                if ((message.id&0x7E8) == 0x7E8) {
                  txLEDState ^= 0x01;
                  digitalWrite(LED_CAN_TX, txLEDState);
                  
                  switch (message.data[2]) {
                    case 0x05:  // Coolant Temp
                      temp = message.data[3];
                      Serial.print("RPM is: ");
                      Serial.println(rpm);
                      break;
                    case 0x0C:  // Engine RPM
                      rpm = ((unsigned int)message.data[4])>>2 + ((unsigned int)message.data[3])<<6;
                      if(rpm > 2000) {
                        if((warningTimestamp - millis()) > 5000) {
                          warningTimestamp = millis();
                        }
                      }
                      Serial.print("RPM is: ");
                      Serial.println(rpm);
                      break;
                    case 0x0D:  // Vehicle Speed
                      mph = message.data[3];
                      Serial.print("KPH is: ");
                      Serial.println(rpm);
                      break;
                    case 0x10:  // MAF Rate
                      break;
                    case 0x2F:  // Fuel Level Input
                      fuel = message.data[3];
                      Serial.print("Fuel is: ");
                      Serial.println(fuel);
                      break;
                  }
                }
	}

      if(intFlag) {
//        Serial.println("Interrupted");
        intFlag = 0;
      }
}

