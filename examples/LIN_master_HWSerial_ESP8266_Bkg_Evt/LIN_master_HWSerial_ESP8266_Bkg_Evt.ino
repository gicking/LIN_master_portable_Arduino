/*********************

Example code for LIN master node with background operation using HardwareSerial

This code runs a LIN master node in "background" operation using HardwareSerial interface. Handler is called by Serial.event()

Supported (=successfully tested) boards:
 - ESP8266 D1 mini        https://www.wemos.cc/en/latest/d1/d1_mini.html

**********************/

// include files
#include "LIN_master_HardwareSerial_ESP8266.h"


// pin to demonstrate background operation
#define PIN_TOGGLE    D1

// indicate LIN return status
#define PIN_ERROR     D2

// pause between LIN frames
#define LIN_PAUSE     100

// skip serial output (for time measurements)
//#define SKIP_CONSOLE


// setup LIN node
LIN_Master_HardwareSerial_ESP8266   LIN(true, "LIN_HW");    // parameter: use alternate Serial2 pins (Rx=D7/Tx=D8), name


// call when byte was received via Serial. This routine is run between each time loop() runs, 
// so using delay inside loop delays response. Multiple bytes of data may be available.
void serialEvent()
{
  // call LIN background handler
  LIN.handler();

} // serialEvent()


// call once
void setup()
{
  // indicate background operation
  pinMode(PIN_TOGGLE, OUTPUT);

  // indicate LIN status via pin
  pinMode(PIN_ERROR, OUTPUT);

  // open LIN connection
  LIN.begin(19200);

  // for output (only) to console
  Serial1.begin(115200);
  while(!Serial1);

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
      Serial1.print(millis());
      Serial1.print("\t");
      Serial1.print(LIN.nameLIN);
      if (Type == LIN_Master_Base::MASTER_REQUEST)
      {
        Serial1.print(" request background: 0x");
        Serial1.println(LIN.getError(), HEX);
      }
      else
      {
        Serial1.print(" reponse background: 0x");
        Serial1.println(LIN.getError(), HEX);
        for (uint8_t i=0; (i < NumData) && (LIN.getError() == LIN_Master_Base::NO_ERROR); i++)
        {
          Serial1.print("\t");        
          Serial1.print((int) i);
          Serial1.print("\t0x");
          Serial1.println((int) Data[i], HEX);
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
