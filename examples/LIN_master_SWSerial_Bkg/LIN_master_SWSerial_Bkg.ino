/*********************

Example code for LIN master node with background operation using SoftwareSerial. 
Note that transmit is always blocking, only receive is in background with CPU availability depending on used CPU  

Optional Tx direction switching for RS485 interface (e.g. MAX485) is by defining 'PIN_TXEN'. 
In this case, permanently enable Rx (REN=GND) for receiving echo

Tested boards:
  - Arduino Mega 2560       https://docs.arduino.cc/hardware/mega-2560/
  x Arduino Due (SoftwareSerial not available)   https://docs.arduino.cc/hardware/due/
  - Arduino Nano Every      https://docs.arduino.cc/hardware/nano-every/
  - Adafruit Trinket        https://www.adafruit.com/product/1501
  - Arduino Uno R4 Minima (blocking)  https://docs.arduino.cc/hardware/uno-r4-minima/ 
  - Arduino Nano ESP32-S3   https://docs.arduino.cc/hardware/nano-esp32/
  - ESP32 WROOM-32UE        https://documentation.espressif.com/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf
  - ESP8266 D1 Mini         https://www.wemos.cc/en/latest/d1/d1_mini.html
  - Nucleo-STM32L432KC      https://www.st.com/en/evaluation-tools/nucleo-l432kc.html

Important: 
  - for ESP8266 during programming and boot, D8(=Tx) must be left open!
  - for supported Rx pins see https://docs.arduino.cc/learn/built-in-libraries/software-serial/

**********************/

// include files
#include "LIN_master_SoftwareSerial.h"

// pause [ms] between LIN frames
#define LIN_FRAME_PERIOD      200

// LIN handler update period [us]
#define LIN_HANDLER_PERIOD    300


////////////////////
// Arduino Mega settings
////////////////////
#if defined(ARDUINO_AVR_MEGA2560)
  #define PIN_LIN_TX      18       // transmit pin for LIN
  #define PIN_LIN_RX      10       // receive pin for LIN. Options: 10, 11, 12, 13, 14, 15, 50, 51, 52, 53, A8, A9, A10, A11, A12, A13, A14, A15
  //#define PIN_TXEN      17         // optional Tx direction pin (=DE) for RS485 physical I/F. Comment out for LIN I/F 
  #define PIN_TOGGLE      30       // pin to demonstrate background operation
  #define PIN_ERROR       32       // indicate LIN return status
  #define SERIAL_CONSOLE  Serial   // serial I/F for console output (comment for no output)

////////////////////
// Arduino Nano Every settings
////////////////////
#elif defined(ARDUINO_AVR_NANO_EVERY)
  #define PIN_LIN_TX      2
  #define PIN_LIN_RX      3        // options: any GPIO
  //#define PIN_TXEN      7
  #define PIN_TOGGLE      4
  #define PIN_ERROR       6
  #define SERIAL_CONSOLE  Serial

////////////////////
// Arduino Uno R4 Minima settings (blocking only)
////////////////////
#elif defined(ARDUINO_UNOR4_MINIMA)
  #define PIN_LIN_TX      1
  #define PIN_LIN_RX      0        // options: D0, D1, D2, D3, D8, D14, D15, A1, A2, A3, A4, A5
  //#define PIN_TXEN      10
  #define PIN_TOGGLE      4
  #define PIN_ERROR       6
  #define SERIAL_CONSOLE  Serial

////////////////////
// Adafruit Trinket settings
////////////////////
#elif defined(ARDUINO_AVR_TRINKET3) || defined(ARDUINO_AVR_TRINKET5)
  #define PIN_LIN_TX      2
  #define PIN_LIN_RX      0        // options: any GPIO
  //#define PIN_TXEN      4
  #define PIN_TOGGLE      1
  #define PIN_ERROR       3
  // Trinket has no HW-Serial!

////////////////////
// Arduino Nano ESP32-S3
////////////////////
#elif defined(ARDUINO_NANO_ESP32)
  #define PIN_LIN_TX      3
  #define PIN_LIN_RX      4        // options: any GPIO
  //#define PIN_TXEN      10
  #define PIN_TOGGLE      5
  #define PIN_ERROR       6
  #define SERIAL_CONSOLE  Serial

////////////////////
// ESP32 WROOM-32UE
////////////////////
#elif defined(ARDUINO_ESP32_WROOM_DA)
  #define PIN_LIN_TX      17
  #define PIN_LIN_RX      16
  //#define PIN_TXEN      21
  #define PIN_TOGGLE      19
  #define PIN_ERROR       18
  #define SERIAL_CONSOLE  Serial

////////////////////
// ESP8266 D1 Mini
////////////////////
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)
  #define PIN_LIN_TX      D8       // must be left open during programming and boot 
  #define PIN_LIN_RX      D7
  //#define PIN_TXEN      D3
  #define PIN_TOGGLE      D1
  #define PIN_ERROR       D2
  #define SERIAL_CONSOLE  Serial1  // Use Tx-only UART1 on pin D4 via UART<->USB adapter

////////////////////
// Nucleo-STM32L432KC
////////////////////
#elif defined(ARDUINO_NUCLEO_L432KC)
  #define PIN_LIN_TX      1
  #define PIN_LIN_RX      0
  //#define PIN_TXEN      5
  #define PIN_TOGGLE      3
  #define PIN_ERROR       4
  #define SERIAL_CONSOLE  Serial

////////////////////
// Board not defined -> error
////////////////////
#else
  #error board not yet supported, exit!
#endif


// setup LIN node. Parameters: Rx, Tx, inverse, name, TxEN
#if defined(PIN_TXEN)
  LIN_master_SoftwareSerial   LIN(PIN_LIN_RX, PIN_LIN_TX, false, "Master", PIN_TXEN);
#else
  LIN_master_SoftwareSerial   LIN(PIN_LIN_RX, PIN_LIN_TX, false, "Master");
#endif


// call once
void setup()
{
  // open optional console
  #if defined(SERIAL_CONSOLE)

    // Nucleo-STM32L432KC, if solder bridges for VCP via STLink have been removed 
    #if defined(ARDUINO_NUCLEO_L432KC) && (1)
      Serial2.setTx(PA_2_ALT1);   // pin A7 on Nucleo-STM32L432KC / uC pin 8
      Serial2.setRx(PA_3_ALT1);   // pin A2 on Nucleo-STM32L432KC / uC pin 9. Optional Rx pin
    #endif 

    SERIAL_CONSOLE.begin(115200);
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
  static uint32_t           lastLINHandler = 0;
  static uint8_t            count = 0;
  uint8_t                   Tx[4] = {0x01, 0x02, 0x03, 0x04};
  LIN_Master_Base::frame_t  Type;
  LIN_Master_Base::error_t  error;  
  uint8_t                   Id;
  uint8_t                   NumData;
  uint8_t                   Data[8];
  

  // toggle pin to show background operation
  digitalWrite(PIN_TOGGLE, !digitalRead(PIN_TOGGLE));

  // periodically call LIN background handler
  if (micros() - lastLINHandler > LIN_HANDLER_PERIOD)
  {
    lastLINHandler = micros();
    LIN.handler();
  }


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
  if (millis() - lastLINFrame > LIN_FRAME_PERIOD)
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