/*********************

Example code for LIN master node with background operation using HardwareSerial

This code runs a LIN master node in "background" operation using HardwareSerial interface. Handler is called by Serial.event()

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
#define LIN_PAUSE     100

// skip serial output (for time measurements)
//#define SKIP_CONSOLE


// setup LIN node
LIN_Master_HardwareSerial   LIN(Serial3, "LIN_HW");             // parameter: HW-interface, name


// call when byte was received via Serial1. This routine is run between each time loop() runs, 
// so using delay inside loop delays response. Multiple bytes of data may be available.
void serialEvent3()
{
  // call LIN background handler
  LIN.handler();

} // serialEvent1()


// call once
void setup()
{
  // indicate background operation
  pinMode(PIN_TOGGLE, OUTPUT);

  // indicate LIN status via pin
  pinMode(PIN_ERROR, OUTPUT);

  // open LIN connection
  LIN.begin(19200);
  
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
  LIN_Master_Base::frame_t   Type;
  uint8_t               Id;
  uint8_t               NumData;
  uint8_t               Data[8];
  

  ///////////////
  // as fast as possible
  ///////////////
  
  // toggle pin to show background operation
  digitalWrite(PIN_TOGGLE, !digitalRead(PIN_TOGGLE));


  ///////////////
  // check if LIN frame has finished
  ///////////////
  if (LIN.getState() == LIN_Master_Base::STATE_DONE)
  {
    // get frame data
    LIN.getFrame(Type, Id, NumData, Data);

    // indicate status via pin
    digitalWrite(PIN_ERROR, LIN.getError());

    // print result
    #if !defined(SKIP_CONSOLE)
      Serial.print(millis());
      Serial.print("\t");
      Serial.print(LIN.nameLIN);
      if (Type == LIN_Master_Base::MASTER_REQUEST)
      {
        Serial.print(" request background: 0x");
        Serial.println(LIN.getError(), HEX);
      }
      else
      {
        Serial.print(" reponse background: 0x");
        Serial.println(LIN.getError(), HEX);
        for (uint8_t i=0; (i < NumData) && (LIN.getError() == LIN_Master_Base::NO_ERROR); i++)
        {
          Serial.print("\t");        
          Serial.print((int) i);
          Serial.print("\t0x");
          Serial.println((int) Data[i], HEX);
        }
      }
    #endif // SKIP_CONSOLE

    // reset state machine & error
    LIN.resetStateMachine();
    LIN.resetError();

  } // if LIN frame finished


  ///////////////
  // SW scheduler for sending/receiving LIN frames
  ///////////////
  if (millis() - lastLINFrame > LIN_PAUSE)
  {
    lastLINFrame = millis();

    // send master request frame (background)
    if (count == 0)
    {
      count++;
      LIN.sendMasterRequest(LIN_Master_Base::LIN_V2, 0x1B, 3, Tx);
    }


    // send slave response frame (background)
    else
    {
      count = 0;
      LIN.receiveSlaveResponse(LIN_Master_Base::LIN_V2, 0x05, 8);
    }
    
  } // SW scheduler

} // loop()
