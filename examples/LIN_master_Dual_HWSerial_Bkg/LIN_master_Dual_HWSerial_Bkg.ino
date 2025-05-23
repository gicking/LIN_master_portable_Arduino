/*********************

Example code for two LIN master nodes with background operation using HardwareSerial

This code runs two LIN master nodes in "background" operation using HardwareSerial interfaces

Note: after starting a frame, LIN handlers must be called every <=500us at least until state has changed from STATE_BREAK to STATE_BODY

Supported (=successfully tested) boards:
 - Arduino Mega 2560      https://store.arduino.cc/products/arduino-mega-2560-rev3
 - Arduino Due            https://store.arduino.cc/products/arduino-due

**********************/

// include files
#include "LIN_master_HardwareSerial.h"


// pin to demonstrate background operation
#define PIN_TOGGLE    30

// indicate LIN1 return status
#define PIN_ERROR1    31

// indicate LIN2 return status
#define PIN_ERROR2    32

// pause [ms] between LIN frames
#define LIN_PAUSE     500

// serial I/F for console output (comment for no output)
#define SERIAL_CONSOLE  Serial
//#define SERIAL_CONSOLE  SerialUSB         // Arduino Due native USB port

// SERIAL_CONSOLE.begin() timeout [ms] (<=0 -> no timeout). Is relevant for native USB ports, if USB is not connected 
#define SERIAL_CONSOLE_BEGIN_TIMEOUT  3000


// setup 2 LIN nodes. Parameters: interface, name, TxEN
LIN_Master_HardwareSerial   LIN1(Serial1, "Master_1");
LIN_Master_HardwareSerial   LIN2(Serial2, "Master_2");


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

  // open LIN interfaces
  LIN1.begin(19200);  
  LIN2.begin(9600);  

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

  // call LIN background handlers
  LIN1.handler();
  LIN2.handler();


  ///////////////
  // check if LIN1 frame has finished
  ///////////////
  if (LIN1.getState() == LIN_Master_Base::STATE_DONE)
  {
    // get frame data & error status
    LIN1.getFrame(Type, Id, NumData, Data);
    error = LIN1.getError();

    // indicate status via pin
    digitalWrite(PIN_ERROR1, error);

    // print result
    #if defined(SERIAL_CONSOLE)
      if (Type == LIN_Master_Base::MASTER_REQUEST)
      {
        SERIAL_CONSOLE.print(LIN1.nameLIN);
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
        SERIAL_CONSOLE.print(LIN1.nameLIN);
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
    LIN1.resetStateMachine();
    LIN1.resetError();

  } // if LIN1 frame finished


  ///////////////
  // check if LIN2 frame has finished
  ///////////////
  if (LIN2.getState() == LIN_Master_Base::STATE_DONE)
  {
    // get frame data & error status
    LIN2.getFrame(Type, Id, NumData, Data);
    error = LIN2.getError();

    // indicate status via pin
    digitalWrite(PIN_ERROR2, error);

    // print result
    #if defined(SERIAL_CONSOLE)
      if (Type == LIN_Master_Base::MASTER_REQUEST)
      {
        SERIAL_CONSOLE.print(LIN2.nameLIN);
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
        SERIAL_CONSOLE.print(LIN2.nameLIN);
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
      LIN1.sendMasterRequest(LIN_Master_Base::LIN_V2, 0x1A, 4, Tx);
    }


    // send LIN1 slave response frame (background)
    else if (count == 1)
    {
      count++;
      LIN1.receiveSlaveResponse(LIN_Master_Base::LIN_V2, 0x05, 6);
    }

    // send LIN2 master request frame (background)
    else if (count == 2)
    {
      count++;
      LIN2.sendMasterRequest(LIN_Master_Base::LIN_V1, 0x1A, 3, Tx);
    }


    // send LIN2 slave response frame (background)
    else
    {
      count = 0;
      LIN2.receiveSlaveResponse(LIN_Master_Base::LIN_V1, 0x06, 8);
    }
    
  } // SW scheduler

} // loop()
