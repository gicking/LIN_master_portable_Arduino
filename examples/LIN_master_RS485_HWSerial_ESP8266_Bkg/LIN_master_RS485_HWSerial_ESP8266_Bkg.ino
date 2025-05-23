/*********************

Example code for LIN master node with background operation using HardwareSerial.
Physical interface is RS485 (e.g. MAX485) with Tx direction switching. Permanently enable Rx (REN=GND) for receiving echo

This code runs a LIN master node in "background" operation using HardwareSerial interface

Note: after starting a frame, LIN.handler() must be called every <=500us at least until state has changed from STATE_BREAK to STATE_BODY

Supported (=successfully tested) boards:
 - ESP8266 D1 mini        https://www.wemos.cc/en/latest/d1/d1_mini.html

**********************/

// include files
#include "LIN_master_HardwareSerial_ESP8266.h"


// pin to switch RS485 Tx direction (=DE)
#define PIN_TXEN      D3

// pin to demonstrate background operation
#define PIN_TOGGLE    D1

// indicate LIN return status
#define PIN_ERROR     D2

// pause [ms] between LIN frames
#define LIN_PAUSE     200

// serial I/F for console output (comment for no output). Use Tx-only UART1 on pin D4 via UART<->USB adapter
#define SERIAL_CONSOLE  Serial1

// SERIAL_CONSOLE.begin() timeout [ms] (<=0 -> no timeout). Is relevant for native USB ports, if USB is not connected 
#define SERIAL_CONSOLE_BEGIN_TIMEOUT  3000


// setup LIN node. Swap Serial pins to use Tx=D8 & Rx=D7. Parameters: swapPins, name, TxEN
LIN_Master_HardwareSerial_ESP8266   LIN(true, "Master", PIN_TXEN);


// call once
void setup()
{
  // open console with timeout
  #if defined(SERIAL_CONSOLE)
    SERIAL_CONSOLE.begin(115200);
    #if defined(SERIAL_CONSOLE_BEGIN_TIMEOUT) && (SERIAL_CONSOLE_BEGIN_TIMEOUT > 0)
      for (uint32_t startMillis = millis(); (!SERIAL_CONSOLE) && (millis() - startMillis < SERIAL_CONSOLE_BEGIN_TIMEOUT); );
    #else
      while (!SERIAL_CONSOLE);
    #endif
  #endif // SERIAL_CONSOLE

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
  static uint32_t           lastLINFrame = 0;
  static uint8_t            count = 0;
  uint8_t                   Tx[4] = {0x01, 0x02, 0x03, 0x04};
  LIN_Master_Base::frame_t  Type;
  LIN_Master_Base::error_t  error;
  uint8_t                   Id;
  uint8_t                   NumData;
  uint8_t                   Data[8];
  

  ///////////////
  // as fast as possible
  ///////////////
  
  // toggle pin to show background operation
  digitalWrite(PIN_TOGGLE, !digitalRead(PIN_TOGGLE));

  // call LIN background handler
  LIN.handler();


  ///////////////
  // check if LIN frame has finished
  ///////////////
  if (LIN.getState() == LIN_Master_Base::STATE_DONE)
  {
    // get frame data & error status
    LIN.getFrame(Type, Id, NumData, Data);
    error = LIN.getError();

    // indicate status via pin
    digitalWrite(PIN_ERROR, error);

    // print result
    #if defined(SERIAL_CONSOLE)
      if (Type == LIN_Master_Base::MASTER_REQUEST)
      {
        SERIAL_CONSOLE.print(LIN.nameLIN);
        SERIAL_CONSOLE.print(", request, ID=0x");
        SERIAL_CONSOLE.print(Id, HEX);
        if (error != LIN_Master_Base::NO_ERROR)
        { 
          SERIAL_CONSOLE.print(", err=0x");
          SERIAL_CONSOLE.println(error, HEX);
        }
        else
        {
          SERIAL_CONSOLE.print(", data=");        
          for (uint8_t i=0; (i < NumData); i++)
          {
            SERIAL_CONSOLE.print("0x");
            SERIAL_CONSOLE.print((int) Data[i], HEX);
            SERIAL_CONSOLE.print(" ");
          }
          SERIAL_CONSOLE.println();
        }
      }
      else
      {
        SERIAL_CONSOLE.print(LIN.nameLIN);
        SERIAL_CONSOLE.print(", response, ID=0x");
        SERIAL_CONSOLE.print(Id, HEX);
        if (error != LIN_Master_Base::NO_ERROR)
        { 
          SERIAL_CONSOLE.print(", err=0x");
          SERIAL_CONSOLE.println(error, HEX);
        }
        else
        {
          SERIAL_CONSOLE.print(", data=");        
          for (uint8_t i=0; (i < NumData); i++)
          {
            SERIAL_CONSOLE.print("0x");
            SERIAL_CONSOLE.print((int) Data[i], HEX);
            SERIAL_CONSOLE.print(" ");
          }
          SERIAL_CONSOLE.println();
        }
      }
    #endif // SERIAL_CONSOLE

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
      LIN.sendMasterRequest(LIN_Master_Base::LIN_V2, 0x1A, 4, Tx);
    }


    // send slave response frame (background)
    else
    {
      count = 0;
      LIN.receiveSlaveResponse(LIN_Master_Base::LIN_V2, 0x05, 6);
    }
    
  } // SW scheduler

} // loop()
