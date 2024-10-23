/*********************

Example code for LIN master node with blocking operation using HardwareSerial

This code runs a LIN master node in blocking operation using HardwareSerial interface

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


// call once
void setup()
{
  // indicate background operation
  pinMode(PIN_TOGGLE, OUTPUT);

  // indicate LIN status via pin
  pinMode(PIN_ERROR, OUTPUT);

  // open LIN interface
  LIN.begin(19200);  

  // for output (only) to console
  Serial1.begin(115200);
  while(!Serial1);

} // setup()


// call repeatedly
void loop()
{
  static uint32_t       tStart;
  uint8_t               Tx[4] = {0x01, 0x02, 0x03, 0x04};
  LIN_Master::frame_t   Type;
  uint8_t               Id;
  uint8_t               NumData;
  uint8_t               Data[8];
  LIN_Master::error_t   error;


  // send master request frame and get result immediately
  error = LIN.sendMasterRequestBlocking(LIN_Master::LIN_V2, 0x1A, 4, Tx);

  // indicate status via pin
  digitalWrite(PIN_ERROR, error);

  // get frame data
  LIN.getFrame(Type, Id, NumData, Data);

  // print result
  #if !defined(SKIP_CONSOLE)
    Serial1.print(millis());
    Serial1.print("\t");
    Serial1.print(LIN.nameLIN);
    Serial1.print(" request blocking: 0x");
    Serial1.println(error, HEX);
  #endif // SKIP_CONSOLE

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
  error = LIN.receiveSlaveResponseBlocking(LIN_Master::LIN_V2, 0x05, 8, Data);

  // indicate status via pin
  digitalWrite(PIN_ERROR, error);

  // get frame data
  LIN.getFrame(Type, Id, NumData, Data);

  // print result
  #if !defined(SKIP_CONSOLE)
    Serial1.print(millis());
    Serial1.print("\t");
    Serial1.print(LIN.nameLIN);
    Serial1.print(" response blocking: 0x");
    Serial1.println(error, HEX);
    for (uint8_t i=0; (i < NumData) && (LIN.getError() == LIN_Master::NO_ERROR); i++)
    {
      Serial1.print("\t");        
      Serial1.print((int) i);
      Serial1.print("\t0x");
      Serial1.println((int) Data[i], HEX);
    }
  #endif // SKIP_CONSOLE

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
