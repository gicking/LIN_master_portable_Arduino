/*********************

Example code for LIN master node with background operation using HardwareSerial

This code runs a LIN master node in "background" operation using HardwareSerial interface

Note: after starting a frame, LIN.handler() must be called every <=500us at least until state has changed from STATE_BREAK to STATE_BODY

Supported (=successfully tested) boards:
 - Arduino Mega 2560      https://store.arduino.cc/products/arduino-mega-2560-rev3
 - Arduino Due            https://store.arduino.cc/products/arduino-due

**********************/

// include files
#include "LIN_master_HardwareSerial.h"


// pin to demonstrate background operation
#define PIN_TOGGLE    30

// pause between LIN frames
#define LIN_PAUSE     100

// skip serial output (for time measurements)
//#define SKIP_CONSOLE


// setup 2 LIN nodes
LIN_Master_HardwareSerial   LIN1(Serial1, "LIN1");             // parameter: HW-interface, name
LIN_Master_HardwareSerial   LIN2(Serial2, "LIN2");             // parameter: HW-interface, name


// call once
void setup()
{
  // indicate background operation
  pinMode(PIN_TOGGLE, OUTPUT);

  // open LIN interfaces
  LIN1.begin(19200);  
  LIN2.begin(9600);  
  
  // for user interaction via console
  Serial.begin(115200);
  while(!Serial);

} // setup()


// call repeatedly
void loop()
{
  static uint32_t       lastLINFrame = 0;
  static uint8_t        count = 0;
  uint8_t               Tx[4] = {0x01, 0x02, 0x03, 0x04};
  LIN_Master::frame_t   Type;
  uint8_t               Id;
  uint8_t               NumData;
  uint8_t               Data[8];
  LIN_Master::error_t   error;
  

  ///////////////
  // as fast as possible
  ///////////////
  
  // toggle pin to show background operation
  digitalWrite(PIN_TOGGLE, !digitalRead(PIN_TOGGLE));

  // call LIN background handlers
  LIN1.handler();
  LIN2.handler();


  ///////////////
  // check if LIN1 frame has finished
  ///////////////
  if (LIN1.getState() == LIN_Master::STATE_DONE)
  {
    // get frame data
    LIN1.getFrame(Type, Id, NumData, Data);

    // print result
    #if !defined(SKIP_CONSOLE)
      Serial.print(millis());
      Serial.print("\t");
      Serial.print(LIN1.nameLIN);
      if (Type == LIN_Master::MASTER_REQUEST)
      {
        Serial.print(" request background: 0x");
        Serial.println(LIN1.getError(), HEX);
      }
      else
      {
        Serial.print(" reponse background: 0x");
        Serial.println(LIN1.getError(), HEX);
        for (uint8_t i=0; (i < NumData) && (LIN1.getError() == LIN_Master::NO_ERROR); i++)
        {
          Serial.print("\t");
          Serial.print((int) i);
          Serial.print("\t0x");
          Serial.println((int) Data[i], HEX);
        }
      }
    #endif // SKIP_CONSOLE

    // reset state machine & error
    LIN1.resetStateMachine();
    LIN1.resetError();

  } // if LIN1 frame finished


  ///////////////
  // check if LIN2 frame has finished
  ///////////////
  if (LIN2.getState() == LIN_Master::STATE_DONE)
  {
    // get frame data
    LIN2.getFrame(Type, Id, NumData, Data);

    // print result
    #if !defined(SKIP_CONSOLE)
      Serial.print(millis());
      Serial.print("\t");
      Serial.print(LIN2.nameLIN);
      if (Type == LIN_Master::MASTER_REQUEST)
      {
        Serial.print(" request background: 0x");
        Serial.println(LIN2.getError(), HEX);
      }
      else
      {
        Serial.print(" reponse background: 0x");
        Serial.println(LIN2.getError(), HEX);
        for (uint8_t i=0; (i < NumData) && (LIN2.getError() == LIN_Master::NO_ERROR); i++)
        {
          Serial.print("\t");        
          Serial.print((int) i);
          Serial.print("\t0x");
          Serial.println((int) Data[i], HEX);
        }
      }
    #endif // SKIP_CONSOLE

    // reset state machine & error
    LIN2.resetStateMachine();
    LIN2.resetError();

  } // if LIN2 frame finished


  ///////////////
  // SW scheduler for sending/receiving LIN frames
  ///////////////
  if (millis() - lastLINFrame > LIN_PAUSE)
  {
    lastLINFrame = millis();

    // send LIN1 master request frame (background)
    if (count == 0)
    {
      count++;
      LIN1.sendMasterRequest(LIN_Master::LIN_V2, 0x1B, 3, Tx);
    }


    // send LIN1 slave response frame (background)
    else if (count == 1)
    {
      count++;
      LIN1.receiveSlaveResponse(LIN_Master::LIN_V2, 0x05, 8);
    }

    // send LIN2 master request frame (background)
    else if (count == 2)
    {
      count++;
      LIN2.sendMasterRequest(LIN_Master::LIN_V1, 0x1A, 3, Tx);
    }


    // send LIN2 slave response frame (background)
    else
    {
      count = 0;
      LIN2.receiveSlaveResponse(LIN_Master::LIN_V1, 0x06, 8);
    }
    
  } // SW scheduler

} // loop()
