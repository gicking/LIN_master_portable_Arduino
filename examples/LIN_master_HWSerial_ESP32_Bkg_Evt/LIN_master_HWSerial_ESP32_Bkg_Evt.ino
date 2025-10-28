/*********************

Example code for LIN master node with background operation using HardwareSerial

This code runs a LIN master node in "background" operation using HardwareSerial interface. Handler is called by Serial.event()

Supported (=successfully tested) boards:
 - Arduino Nano ESP32           https://docs.arduino.cc/hardware/nano-esp32/
 - Espressif ESP32-WROOM-32UE   https://www.etechnophiles.com/esp32-dev-board-pinout-specifications-datasheet-and-schematic/

**********************/

// include files
#include "LIN_master_HardwareSerial_ESP32.h"


// Arduino Nano ESP32 board (using Arduino ESP32 core)
#if defined(ARDUINO_NANO_ESP32)

  // LIN transmit pin
  #define PIN_LIN_TX    3

  // LIN receive pin
  #define PIN_LIN_RX    4

  // pin to demonstrate background operation
  #define PIN_TOGGLE    5

  // indicate LIN return status
  #define PIN_ERROR     6

// Espressif ESP32-WROOM-32UE board (using Espressif ESP32 core)
#elif defined(ARDUINO_ESP32_WROOM_DA)

  // LIN transmit pin
  #define PIN_LIN_TX    17

  // LIN receive pin
  #define PIN_LIN_RX    16

  // pin to demonstrate background operation
  #define PIN_TOGGLE    19

  // indicate LIN return status
  #define PIN_ERROR     18

// not tested -> throw error
#else
  #error board not yet tested -> define board pins, check for incompatibilities!
#endif


// pause [ms] between LIN frames
#define LIN_PAUSE     200

// serial I/F for console output (comment for no output)
#define SERIAL_CONSOLE  Serial

// SERIAL_CONSOLE.begin() timeout [ms] (<=0 -> no timeout). Is relevant for native USB ports, if USB is not connected 
#define SERIAL_CONSOLE_BEGIN_TIMEOUT  3000


// setup LIN node. Parameters: interface, Rx, Tx, name, TxEN
LIN_Master_HardwareSerial_ESP32   LIN(Serial2, PIN_LIN_RX, PIN_LIN_TX, "Master");


// call when byte was received via Serial2. This routine is run between each time loop() runs, 
// so using delay inside loop delays response. Multiple bytes of data may be available.
void serialEvent2()
{
  // call LIN background handler
  LIN.handler();

} // serialEvent2()


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
