/*********************

Example code for LIN master node with blocking operation using HardwareSerial

This code runs a LIN master node in blocking operation using HardwareSerial interface

Supported (=successfully tested) boards:
 - Arduino Mega 2560      https://store.arduino.cc/products/arduino-mega-2560-rev3
 - Arduino Due            https://store.arduino.cc/products/arduino-due

**********************/

// include files
#include "LIN_master_HardwareSerial.h"


// pin to demonstrate background operation
#define PIN_TOGGLE    30

// indicate LIN return status
#define PIN_ERROR     32

// pause between LIN frames
#define LIN_PAUSE     200

// serial I/F for debug output (comment for no output)
#define SERIAL_DEBUG  Serial


// setup LIN node
LIN_Master_HardwareSerial   LIN(Serial1, "Master");


// call once
void setup()
{
  // for debug output
  #if defined(SERIAL_DEBUG)
    SERIAL_DEBUG.begin(115200);
    while(!SERIAL_DEBUG);
  #endif // SERIAL_DEBUG
  
  // indicate background operation
  pinMode(PIN_TOGGLE, OUTPUT);

  // indicate LIN status via pin
  pinMode(PIN_ERROR, OUTPUT);

  // open LIN interface
  LIN.begin(19200);

} // setup()


// call repeatedly
void loop()
{
  static uint32_t           tStart;
  uint8_t                   Tx[4] = {0x01, 0x02, 0x03, 0x04};
  LIN_Master_Base::frame_t  Type;
  LIN_Master_Base::error_t  error;  
  uint8_t                   Id;
  uint8_t                   NumData;
  uint8_t                   Data[8];


  // send master request frame and get result immediately
  error = LIN.sendMasterRequestBlocking(LIN_Master_Base::LIN_V2, 0x1A, 4, Tx);

  // indicate status via pin
  digitalWrite(PIN_ERROR, error);

  // get frame data
  LIN.getFrame(Type, Id, NumData, Data);

  // print result
  #if defined(SERIAL_DEBUG)
    SERIAL_DEBUG.print(LIN.nameLIN);
    SERIAL_DEBUG.print(", request, ID=0x");
    SERIAL_DEBUG.print(Id, HEX);
    if (error != LIN_Master_Base::NO_ERROR)
    { 
      SERIAL_DEBUG.print(", err=0x");
      SERIAL_DEBUG.println(error, HEX);
    }
    else
    {
      SERIAL_DEBUG.print(", data=");        
      for (uint8_t i=0; (i < NumData); i++)
      {
        SERIAL_DEBUG.print("0x");
        SERIAL_DEBUG.print((int) Data[i], HEX);
        SERIAL_DEBUG.print(" ");
      }
      SERIAL_DEBUG.println();
    }
  #endif // SERIAL_DEBUG

  // reset state machine & error
  LIN.resetStateMachine();
  LIN.resetError();


  // wait a bit. Toggle pin to show CPU load
  tStart = millis();
  while (millis() - tStart < LIN_PAUSE)
  {
    digitalWrite(PIN_TOGGLE, !digitalRead(PIN_TOGGLE));
  }


  // send/receive slave response frame and get result immediately
  error = LIN.receiveSlaveResponseBlocking(LIN_Master_Base::LIN_V2, 0x05, 6, Data);

  // indicate status via pin
  digitalWrite(PIN_ERROR, error);

  // get frame data
  LIN.getFrame(Type, Id, NumData, Data);

  // print result
  #if defined(SERIAL_DEBUG)
    SERIAL_DEBUG.print(LIN.nameLIN);
    SERIAL_DEBUG.print(", response, ID=0x");
    SERIAL_DEBUG.print(Id, HEX);
    if (error != LIN_Master_Base::NO_ERROR)
    { 
      SERIAL_DEBUG.print(", err=0x");
      SERIAL_DEBUG.println(error, HEX);
    }
    else
    {
      SERIAL_DEBUG.print(", data=");        
      for (uint8_t i=0; (i < NumData); i++)
      {
        SERIAL_DEBUG.print("0x");
        SERIAL_DEBUG.print((int) Data[i], HEX);
        SERIAL_DEBUG.print(" ");
      }
      SERIAL_DEBUG.println();
    }
  #endif // SERIAL_DEBUG
  
  // reset state machine & error
  LIN.resetStateMachine();
  LIN.resetError();


  // wait a bit. Toggle pin to show CPU load
  tStart = millis();
  while (millis() - tStart < LIN_PAUSE)
  {
    digitalWrite(PIN_TOGGLE, !digitalRead(PIN_TOGGLE));
  }

} // loop()
