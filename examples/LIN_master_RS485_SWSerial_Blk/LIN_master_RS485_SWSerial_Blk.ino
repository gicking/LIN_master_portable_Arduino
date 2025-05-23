/*********************

Example code for LIN master node with blocking operation using SoftwareSerial.
Physical interface is RS485 (e.g. MAX485) with Tx direction switching. Permanently enable Rx (REN=GND) for receiving echo

This code runs a LIN master node in blocking operation using SoftwareSerial interface

Supported (=successfully tested) boards + MAX485 (requires 5V board!):
 - Arduino Mega 2560      https://store.arduino.cc/products/arduino-mega-2560-rev3
 - Adafruit Trinket       https://www.adafruit.com/product/1501
 - ESP8266 D1 mini        https://www.wemos.cc/en/latest/d1/d1_mini.html
 - ESP32 Wroom-32UE       https://www.etechnophiles.com/esp32-dev-board-pinout-specifications-datasheet-and-schematic/

**********************/

// include files
#include "LIN_master_SoftwareSerial.h"


// board pin definitions. Note: for supported Rx pins see https://docs.arduino.cc/learn/built-in-libraries/software-serial/
#if defined(ARDUINO_AVR_MEGA2560)
  #define PIN_LIN_TX      18        // transmit pin for LIN
  #define PIN_LIN_RX      10        // receive pin for LIN
  #define PIN_TXEN        17        // pin to switch RS485 Tx direction (=DE)
  #define PIN_TOGGLE      30        // pin to demonstrate background operation
  #define PIN_ERROR       32        // indicate LIN return status
  #define SERIAL_CONSOLE  Serial    // serial I/F for console output (comment for no output)
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)
  #define PIN_LIN_TX      D8
  #define PIN_LIN_RX      D7
  #define PIN_TXEN        D3
  #define PIN_TOGGLE      D1
  #define PIN_ERROR       D2
  #define SERIAL_CONSOLE  Serial1   // Use Tx-only UART1 on pin D4 via UART<->USB adapter
#elif defined(ARDUINO_ESP32_WROOM_DA) || defined(ARDUINO_NANO_ESP32)
  #define PIN_LIN_TX    17
  #define PIN_LIN_RX    16
  #define PIN_TXEN      21
  #define PIN_TOGGLE    19
  #define PIN_ERROR     18
  #define SERIAL_CONSOLE	Serial
#elif defined(ARDUINO_AVR_TRINKET3) || defined(ARDUINO_AVR_TRINKET5)
  #define PIN_LIN_TX    2
  #define PIN_LIN_RX    0
  #define PIN_TXEN      4
  #define PIN_TOGGLE    1
  #define PIN_ERROR     3
  // Trinket has no HW-Serial!
#else
  #error adapt parameters to board   
#endif

// pause [ms] between LIN frames
#define LIN_PAUSE       200

// SERIAL_CONSOLE.begin() timeout [ms] (<=0 -> no timeout). Is relevant for native USB ports, if USB is not connected 
#define SERIAL_CONSOLE_BEGIN_TIMEOUT  3000


// setup LIN node. Parameters: Rx, Tx, inverse, name, TxEN
LIN_Master_SoftwareSerial   LIN(PIN_LIN_RX, PIN_LIN_TX, false, "Master", PIN_TXEN);


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
  static uint32_t       tStart;
  uint8_t               Tx[4] = {0x01, 0x02, 0x03, 0x04};
  LIN_Master_Base::frame_t   Type;
  uint8_t               Id;
  uint8_t               NumData;
  uint8_t               Data[8];
  LIN_Master_Base::error_t   error;


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
  while (millis() - tStart < LIN_PAUSE)
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
  while (millis() - tStart < LIN_PAUSE)
  {
    digitalWrite(PIN_TOGGLE, !digitalRead(PIN_TOGGLE));
  }

} // loop()
