/*********************

Example code for LIN master node with blocking operation using HardwareSerial.

Optional Tx direction switching for RS485 interface (e.g. MAX485) is by defining 'PIN_TXEN'. 
In this case, permanently enable Rx (REN=GND) for receiving echo

This code runs a LIN master node in blocking operation using HardwareSerial interface

Tested boards:
  - Arduino Mega 2560       https://docs.arduino.cc/hardware/mega-2560/
  - Arduino Due             https://docs.arduino.cc/hardware/due/
  - Arduino Nano Every      https://docs.arduino.cc/hardware/nano-every/
  x - Arduino Uno R4 Minima   https://docs.arduino.cc/hardware/uno-r4-minima/
  - Arduino Nano ESP32-S3   https://docs.arduino.cc/hardware/nano-esp32/
  - ESP32 WROOM-32UE        https://documentation.espressif.com/esp32-wroom-32e_esp32-wroom-32ue_datasheet_en.pdf
  - ESP8266 D1 Mini         https://www.wemos.cc/en/latest/d1/d1_mini.html
  x - Nucleo-STM32L432KC      https://www.st.com/en/evaluation-tools/nucleo-l432kc.html

**********************/

// pause [ms] between LIN frames
#define LIN_FRAME_PERIOD      200


////////////////////
// Arduino Mega settings
////////////////////
#if defined(ARDUINO_AVR_MEGA2560)

  #include "LIN_master_HardwareSerial.h"                // matching LIN master header

  //#define PIN_TXEN            17                        // optional Tx direction pin (=DE) for RS485 physical I/F. Comment out for LIN I/F 
  #define PIN_TOGGLE          30                        // pin to show CPU idle
  #define PIN_ERROR           32                        // LIN error status pin (high=error)
  #define SERIAL_CONSOLE      Serial                    // serial I/F for console output (comment for no output)

  // setup LIN node. Parameters: interface, name, TxEN
  #if defined(PIN_TXEN)
    LIN_Master_HardwareSerial   LIN(Serial1, "Master", PIN_TXEN);
  #else
    LIN_Master_HardwareSerial   LIN(Serial1, "Master");
  #endif


////////////////////
// Arduino Due settings
////////////////////
#elif defined(ARDUINO_SAM_DUE)

  #include "LIN_master_HardwareSerial.h"                // matching LIN master header

  //#define PIN_TXEN            17                        // optional Tx direction pin (=DE) for RS485 physical I/F. Comment out for LIN I/F 
  #define PIN_TOGGLE          30                        // pin to show CPU idle
  #define PIN_ERROR           32                        // LIN error status pin (high=error)
  #define SERIAL_CONSOLE      Serial                    // serial I/F for console output (comment for no output). Use SerialUSB for native USB port

  // setup LIN node. Parameters: interface, name, TxEN
  #if defined(PIN_TXEN)
    LIN_Master_HardwareSerial   LIN(Serial1, "Master", PIN_TXEN);
  #else
    LIN_Master_HardwareSerial   LIN(Serial1, "Master");
  #endif


////////////////////
// Arduino Nano Every settings
////////////////////
#elif defined(ARDUINO_AVR_NANO_EVERY)

  #include "LIN_master_HardwareSerial.h"                // matching LIN master header

  //#define PIN_TXEN            7                         // optional Tx direction pin (=DE) for RS485 physical I/F. Comment out for LIN I/F 
  #define PIN_TOGGLE          4                         // pin to show CPU idle
  #define PIN_ERROR           6                         // LIN error status pin (high=error)
  #define SERIAL_CONSOLE      Serial                    // serial I/F for console output (comment for no output)

  // setup LIN node. Parameters: interface, name, TxEN
  #if defined(PIN_TXEN)
    LIN_Master_HardwareSerial   LIN(Serial1, "Master", PIN_TXEN);
  #else
    LIN_Master_HardwareSerial   LIN(Serial1, "Master");
  #endif


////////////////////
// Arduino Uno R4 Minima settings
////////////////////
#elif defined(ARDUINO_UNOR4_MINIMA)

  #include "LIN_master_HardwareSerial.h"                // matching LIN master header

  //#define PIN_TXEN            10                        // optional Tx direction pin (=DE) for RS485 physical I/F. Comment out for LIN I/F 
  #define PIN_TOGGLE          4                         // pin to show CPU idle
  #define PIN_ERROR           6                         // LIN error status pin (high=error)
  #define SERIAL_CONSOLE      Serial                    // serial I/F for console output (comment for no output)

  // setup LIN node. Parameters: interface, name, TxEN
  #if defined(PIN_TXEN)
    LIN_Master_HardwareSerial   LIN(Serial1, "Master", PIN_TXEN);
  #else
    LIN_Master_HardwareSerial   LIN(Serial1, "Master");
  #endif


////////////////////
// Arduino Nano ESP32 board settings (using Arduino ESP32 core)
////////////////////
#elif defined(ARDUINO_NANO_ESP32)

  #include "LIN_master_HardwareSerial_ESP32.h"          // matching LIN master header

  //#define PIN_TXEN            10                        // optional Tx direction pin (=DE) for RS485 physical I/F. Comment out for LIN I/F 
  #define PIN_LIN_TX          3                         // LIN transmit pin
  #define PIN_LIN_RX          4                         // LIN receive pin
  #define PIN_TOGGLE          5                         // pin to show CPU idle
  #define PIN_ERROR           6                         // LIN error status pin (high=error)
  #define SERIAL_CONSOLE      Serial                    // serial I/F for console output (comment for no output)

  // setup LIN node. Parameters: interface, Rx, Tx, name, TxEN
  #if defined(PIN_TXEN)
    LIN_Master_HardwareSerial_ESP32   LIN(Serial2, PIN_LIN_RX, PIN_LIN_TX, "Master", PIN_TXEN);
  #else
    LIN_Master_HardwareSerial_ESP32   LIN(Serial2, PIN_LIN_RX, PIN_LIN_TX, "Master");
  #endif


////////////////////
// Espressif ESP32-WROOM-32UE board settings (using Espressif ESP32 core)
////////////////////
#elif defined(ARDUINO_ESP32_WROOM_DA)

  #include "LIN_master_HardwareSerial_ESP32.h"          // matching LIN master header

  //#define PIN_TXEN            21                        // optional Tx direction pin (=DE) for RS485 physical I/F. Comment out for LIN I/F 
  #define PIN_LIN_TX          17                        // LIN transmit pin
  #define PIN_LIN_RX          16                        // LIN receive pin
  #define PIN_TOGGLE          19                        // pin to show CPU idle
  #define PIN_ERROR           18                        // LIN error status pin (high=error)
  #define SERIAL_CONSOLE      Serial                    // serial I/F for console output (comment for no output)

  // setup LIN node. Parameters: interface, Rx, Tx, name, TxEN
  #if defined(PIN_TXEN)
    LIN_Master_HardwareSerial_ESP32   LIN(Serial2, PIN_LIN_RX, PIN_LIN_TX, "Master", PIN_TXEN);
  #else
    LIN_Master_HardwareSerial_ESP32   LIN(Serial2, PIN_LIN_RX, PIN_LIN_TX, "Master");
  #endif


////////////////////
// Wemos ESP8266 D1 Mini settings
////////////////////
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)

  #include "LIN_master_HardwareSerial_ESP8266.h"        // matching LIN master header

  //#define PIN_TXEN            D3                        // optional Tx direction pin (=DE) for RS485 physical I/F. Comment out for LIN I/F 
  #define PIN_TOGGLE          D1                        // pin to show CPU idle
  #define PIN_ERROR           D2                        // LIN error status pin (high=error)
  #define SERIAL_CONSOLE      Serial1                   // serial I/F for console output (comment for no output). Use Tx-only UART1 on pin D4 via UART<->USB adapter

  // setup LIN node. Swap Serial pins to use Tx=D8 & Rx=D7. Parameters: swapPins, name, TxEN
  #if defined(PIN_TXEN)
    LIN_Master_HardwareSerial_ESP8266   LIN(true, "Master", PIN_TXEN);
  #else
    LIN_Master_HardwareSerial_ESP8266   LIN(true, "Master");
  #endif


////////////////////
// Nucleo-STM32L432KC settings
////////////////////
#elif defined(ARDUINO_NUCLEO_L432KC)

  #include "LIN_master_HardwareSerial.h"                // matching LIN master header

  //#define PIN_TXEN            D5                        // optional Tx direction pin (=DE) for RS485 physical I/F. Comment out for LIN I/F 
  #define PIN_TOGGLE          D3                        // pin to show CPU idle
  #define PIN_ERROR           D4                        // LIN error status pin (high=error)
  #define SERIAL_CONSOLE      Serial2                   // serial I/F for console output (comment for no output)

  HardwareSerial              Serial1(PA_10, PA_9);       // Serial1 is not allocated by default. Parametes: Rx=D0, Tx=D1

  // setup LIN node. Parameters: interface, name, TxEN
  #if defined(PIN_TXEN)
    LIN_Master_HardwareSerial   LIN(Serial1, "Master", PIN_TXEN);
  #else
    LIN_Master_HardwareSerial   LIN(Serial1, "Master");
  #endif


// board not yet included
#else
  #error board not yet supported, exit!
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
  #if defined(SERIAL_CONSOLE)
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
  #endif // SERIAL_CONSOLE

  // reset state machine & error
  LIN.resetStateMachine();
  LIN.resetError();


  // wait a bit. Toggle pin to show CPU load
  tStart = millis();
  while (millis() - tStart < LIN_FRAME_PERIOD)
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
  #if defined(SERIAL_CONSOLE)
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
  #endif // SERIAL_CONSOLE
  
  // reset state machine & error
  LIN.resetStateMachine();
  LIN.resetError();


  // wait a bit. Toggle pin to show CPU load
  tStart = millis();
  while (millis() - tStart < LIN_FRAME_PERIOD)
  {
    digitalWrite(PIN_TOGGLE, !digitalRead(PIN_TOGGLE));
  }

} // loop()
