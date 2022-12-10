/*********************

Example code for LIN master node with background operation using HardwareSerial

This code runs a LIN master node in "background" operation using HardwareSerial interface

Note: after starting a frame, LIN.handler() must be called every <=500us at least until state has changed from STATE_BREAK to STATE_BODY

Supported (=successfully tested) boards:
 - ESP32 Wroom-32U        https://www.etechnophiles.com/esp32-dev-board-pinout-specifications-datasheet-and-schematic/

**********************/

// include files
#include <Ticker.h>
#include "LIN_master_HardwareSerial_ESP32.h"


// board pin definitions (GPIOn is referred to as n)
#define PIN_TOGGLE    19        // pin to demonstrate background operation
#define PIN_ERROR     23        // indicate LIN return status
#define PIN_LIN_RX    16        // receive pin for LIN
#define PIN_LIN_TX    17        // transmit pin for LIN

// pause between LIN frames
#define LIN_PAUSE     100

// skip serial output (for time measurements)
//#define SKIP_CONSOLE


// setup LIN node
LIN_Master_HardwareSerial_ESP32   LIN(Serial2, PIN_LIN_RX, PIN_LIN_TX, "LIN_HW");    // parameter: interface, Rx, Tx, name

// task scheduler for background
Ticker taskHandler;
Ticker taskScheduler;
Ticker taskPrint;


// send & receive frames
void LIN_scheduler()
{
  static uint8_t  count = 0;
  uint8_t         Tx[4] = {0x01, 0x02, 0x03, 0x04};

  // send master request frame (background)
  if (count == 0)
  {
    count++;
    LIN.sendMasterRequest(LIN_Master::LIN_V2, 0x1B, 3, Tx);
  }


  // send slave response frame (background)
  else
  {
    count = 0;
    LIN.receiveSlaveResponse(LIN_Master::LIN_V2, 0x05, 8);
  }

} // LIN_scheduler()


// call LIN background handler
void LIN_handler()
{
  LIN.handler();
}


// print result of LIN frame
void LIN_print()
{
  LIN_Master::frame_t   Type;
  uint8_t               Id;
  uint8_t               NumData;
  uint8_t               Data[8];
  LIN_Master::error_t   error;
  

  // if LIN frame has finished
  if (LIN.getState() == LIN_Master::STATE_DONE)
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
      if (Type == LIN_Master::MASTER_REQUEST)
      {
        Serial.print(" request background: 0x");
        Serial.println(LIN.getError(), HEX);
      }
      else
      {
        Serial.print(" reponse background: 0x");
        Serial.println(LIN.getError(), HEX);
        for (uint8_t i=0; (i < NumData) && (LIN.getError() == LIN_Master::NO_ERROR); i++)
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

  } // state == DONE

  // if LIN frame is ongoing -> re-try again in a few ms
  else
  {
    // print error
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(LIN.nameLIN);
    Serial.println(" frame ongoing");

    // try again in a few ms  
    taskPrint.detach();
    delay(10);
    taskPrint.attach_ms(LIN_PAUSE, LIN_print);

  } // state != DONE

} // LIN_print()


// call once
void setup()
{
  // indicate background operation
  pinMode(PIN_TOGGLE, OUTPUT);

  // indicate LIN status via pin
  pinMode(PIN_ERROR, OUTPUT);

  // for output (only) to console
  Serial.begin(115200);
  while(!Serial);

  // open LIN interface
  LIN.begin(19200);  

  // add LIN background tasks
  taskHandler.attach_ms(1, LIN_handler);                // LIN background handler
  taskScheduler.attach_ms(LIN_PAUSE, LIN_scheduler);    // start frames
  delay(20);
  taskPrint.attach_ms(LIN_PAUSE, LIN_print);            // print result of LIN frame (start task with delay!)

} // setup()


// call repeatedly
void loop()
{
  // toggle pin to show background operation
  digitalWrite(PIN_TOGGLE, !digitalRead(PIN_TOGGLE));

} // loop()
