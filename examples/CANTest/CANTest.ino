/*
  MCP2515 CAN Interface Using SPI
  
  Author: David Harding
  
  Created: 11/08/2010
  Modified: 6/26/12 by RechargeCar Inc.
  
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

#include <SPI.h> // Arduino SPI Library
#include <MCP2515.h>

// Pin definitions specific to how the MCP2515 is wired up.
#define CS_PIN    85
#define RESET_PIN  7
#define INT_PIN    84

// Create CAN object with pins as defined
MCP2515 CAN(CS_PIN, RESET_PIN, INT_PIN);

void setup() {
  Serial.begin(115200);
  
  Serial.println("Initializing ...");

  // Set up SPI Communication
  // dataMode can be SPI_MODE0 or SPI_MODE3 only for MCP2515
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.begin();
  
  // Initialise MCP2515 CAN controller at the specified speed and clock frequency
  // In this case 125kbps with a 16MHz oscillator
  // (Note:  This is the oscillator attached to the MCP2515, not the Arduino oscillaltor)
  if(CAN.Init(250,16))
  {
    Serial.println("MCP2515 Init OK ...");
  } else {
    Serial.println("MCP2515 Init Failed ...");
  }

  Serial.println("Ready ...");
}

byte i=0;

// CAN message frame (actually just the parts that are exposed by the MCP2515 RX/TX buffers)
Frame message;

void loop() {
  
  message.id = 0;
  
  // This implementation utilizes the MCP2515 INT pin to flag received messages or other events
  if(CAN.Interrupt()) {
    // determine which interrupt flags have been set
    byte interruptFlags = CAN.Read(CANINTF);
    
    if(interruptFlags & RX0IF) {
      // read from RX buffer 0
      message = CAN.ReadBuffer(RXB0);
    }
    if(interruptFlags & RX1IF) {
      // read from RX buffer 1
      message = CAN.ReadBuffer(RXB1);
      // (this is poor code as clearly if two messages are received then the second will overwrite the first)
    }
    if(interruptFlags & TX0IF) {
      // TX buffer 0 sent
    }
    if(interruptFlags & TX1IF) {
      // TX buffer 1 sent
    }
    if(interruptFlags & TX2IF) {
      // TX buffer 2 sent
    }
    if(interruptFlags & ERRIF) {
      // error handling code
    }
    if(interruptFlags & MERRF) {
      // error handling code
      // if TXBnCTRL.TXERR set then transmission error
      // if message is lost TXBnCTRL.MLOA will be set
    }
 }
  
  if(message.id>0) {
    // Print message
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

    // Send out a return message for each one received
    // Simply increment message id and data bytes to show proper transmission
    // Note:  Please see explanation at top of sketch.  You might want to comment this out!
   // message.id++;
 //   for(i=0;i<message.dlc;i++) {
  //    message.data[i]++;
  //  }
 //   CAN.LoadBuffer(TXB0, message);
 //   CAN.SendBuffer(TXB0);
  }
}
