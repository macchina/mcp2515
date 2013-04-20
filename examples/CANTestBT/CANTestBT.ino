/*
  MCP2515 CAN Interface Using SPI
  Demonstrates CAN Interface use, and has a 
  selectable serial diagnostic output, for output flexibility
  such as Bluetooth Serial Profile shields
  
  Author: David Harding
  
  Created: 11/08/2010
  Modified: 6/26/12 and later by RechargeCar Inc.
  
  For further information see:
  
  http://ww1.microchip.com/downloads/en/DeviceDoc/21801e.pdf
  http://en.wikipedia.org/wiki/CAN_bus

  The MCP2515 Library files also contain important information.
  
  This sketch is configured to work with the 'Macchina' Automotive Interface board 
  manufactured by RechargeCar Inc. CS_PIN and INT_PIN are specific to this board.
  
  This sketch shows the most basic of steps to send and receive CAN messages.
  
  NOTE!!!  If you use this sketch to test on a live system I suggest that you comment out the
  send messages lines unless you are certain that they will have no detrimental effect! 


  This example code is in the public domain.
  
*/

#include <SPI.h>
#include <MCP2515.h>

// Pin definitions specific to how the MCP2515 is wired up.
#define CS_PIN    85
#define RESET_PIN  7
#define INT_PIN    84


#define DEBUG_ENABLED  1
// Change these to whatever UART you want to use
// The normal Arduino diagnostic serial port is just "Serial" 
#define blueToothSerial Serial3 
#define SerialOut Serial3


// Create CAN object with pins as defined
MCP2515 CAN(CS_PIN, RESET_PIN, INT_PIN);

void CANHandler() {
	CAN.intHandler();
}

void setup() {
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
  if(CAN.Init(250,16)) {
    Serial.println("MCP2515 Init OK ...");
  }
  else {
    Serial.println("MCP2515 Init Failed ...");
  }
	
  attachInterrupt(6, CANHandler, FALLING);
  CAN.InitFilters(false);
  CAN.SetRXMask(MASK0, 0x7F8, 0); //match all but bottom four bits
  CAN.SetRXFilter(FILTER0, 0x7E8, 0); // Allows 0x7E8 through 0x7EF, 
                                      // to allow responses from up to 8 ECUs,
                                      // per the 2008 OBD requirements

  setupBlueToothConnection();

  Serial.println("Ready ...");
}

byte i=0;
char recvChar;

// CAN message frame (actually just the parts that are exposed by the MCP2515 RX/TX buffers)
Frame message;

void loop() {	
  if (CAN.GetRXFrame(message)) {
    // Print message
    SerialOut.print("ID: ");
    SerialOut.println(message.id,HEX);
    SerialOut.print("Extended: ");
    if(message.ide) {
      Serial.println("Yes");
    }
    else {
      SerialOut.println("No");
    }
    SerialOut.print("DLC: ");
    SerialOut.println(message.dlc,DEC);
    for(i=0; i<message.dlc; i++) {
      SerialOut.print(message.data[i],HEX);
      SerialOut.print(" ");
    }
    SerialOut.println();
    SerialOut.println();

    // Send out a return message for each one received
    // Simply increment message id and data bytes to show proper transmission
    // Note: this will double the traffic on the network (provided it passes the filter above)
    message.id++;
    for(i=0;i<message.dlc;i++) {
      message.data[i]++;
    }
    CAN.EnqueueTX(message);
  }
}

void setupBlueToothConnection()
{
  blueToothSerial.begin(38400); //Set BluetoothBee BaudRate to default baud rate 38400
  blueToothSerial.print("\r\n+STWMOD=0\r\n"); //set the bluetooth work in slave mode
  blueToothSerial.print("\r\n+STNA=SeeedBTSlave\r\n"); //set the bluetooth name as "SeeedBTSlave"
  blueToothSerial.print("\r\n+STOAUT=1\r\n"); // Permit Paired device to connect me
  blueToothSerial.print("\r\n+STAUTO=0\r\n"); // Auto-connection should be forbidden here
  delay(2000); // This delay is required.
  blueToothSerial.print("\r\n+INQ=1\r\n"); //make the slave bluetooth inquirable 
  Serial.println("The slave bluetooth is inquirable!");
  delay(2000); // This delay is required.
  blueToothSerial.flush();
}
