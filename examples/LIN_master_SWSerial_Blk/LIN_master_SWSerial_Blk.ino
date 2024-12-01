/*********************

Example code for LIN master node with blocking operation using SoftwareSerial

This code runs a LIN master node in blocking operation using SoftwareSerial interface

Supported (=successfully tested) boards:
 - Arduino Mega 2560      https://store.arduino.cc/products/arduino-mega-2560-rev3
 - Adafruit Trinket       https://www.adafruit.com/product/1501
 - ESP8266 D1 mini        https://www.wemos.cc/en/latest/d1/d1_mini.html
 - ESP32 Wroom-32UE       https://www.etechnophiles.com/esp32-dev-board-pinout-specifications-datasheet-and-schematic/

**********************/

// include files
#include "LIN_master_SoftwareSerial.h"


// board pin definitions
#if defined(ARDUINO_AVR_MEGA2560)
  #define PIN_TOGGLE    30        // pin to demonstrate background operation
  #define PIN_ERROR     32        // indicate LIN return status
  #define PIN_LIN_RX    10        // receive pin for LIN
  #define PIN_LIN_TX    14        // transmit pin for LIN
#elif defined(ARDUINO_ESP8266_WEMOS_D1MINI)
  #define PIN_TOGGLE    D1
  #define PIN_ERROR     D2
  #define PIN_LIN_RX    D7
  #define PIN_LIN_TX    D8
#elif defined(ARDUINO_ESP32_WROOM_DA)
  #define PIN_TOGGLE    19        // pin to demonstrate background operation
  #define PIN_ERROR     23        // indicate LIN return status
  #define PIN_LIN_RX    16        // receive pin for LIN
  #define PIN_LIN_TX    17        // transmit pin for LIN
#elif defined(ARDUINO_AVR_TRINKET3)
  #define PIN_TOGGLE    1
  #define PIN_ERROR     3
  #define PIN_LIN_RX    0
  #define PIN_LIN_TX    2
#else
  #error adapt parameters to board   
#endif

// pause between LIN frames
#define LIN_PAUSE       100

// skip serial output (for time measurements)
//#define SKIP_CONSOLE


// setup LIN node. Note: not all pins support Rx!
LIN_Master_SoftwareSerial   LIN(PIN_LIN_RX, PIN_LIN_TX, false, "LIN_SW");       // parameter: Rx, Tx, inverseLogic, name


// call once
void setup()
{
  // indicate background operation
  pinMode(PIN_TOGGLE, OUTPUT);

  // indicate LIN status via pin
  pinMode(PIN_ERROR, OUTPUT);

  // open LIN connection
  LIN.begin(19200);

  // for user interaction via console (Trinket has no Serial)
  #if !defined(ARDUINO_AVR_TRINKET3)
    Serial.begin(115200);
    while(!Serial);
  #endif

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

  // get frame data
  LIN.getFrame(Type, Id, NumData, Data);

  // print result (Trinket has no Serial)
  #if !defined(ARDUINO_AVR_TRINKET3) && !defined(SKIP_CONSOLE)
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(LIN.nameLIN);
    Serial.print(" request blocking: 0x");
    Serial.println(error, HEX);
  #endif

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
  error = LIN.receiveSlaveResponseBlocking(LIN_Master_Base::LIN_V2, 0x05, 8, Data);

  // indicate status via pin
  digitalWrite(PIN_ERROR, error);

  // get frame data
  LIN.getFrame(Type, Id, NumData, Data);

  // print result (Trinket has no Serial)
  #if !defined(ARDUINO_AVR_TRINKET3) && !defined(SKIP_CONSOLE)
    Serial.print(millis());
    Serial.print("\t");
    Serial.print(LIN.nameLIN);
    Serial.print(" response blocking: 0x");
    Serial.println(error, HEX);
    for (uint8_t i=0; (i < NumData) && (LIN.getError() == LIN_Master_Base::NO_ERROR); i++)
    {
      Serial.print("\t");        
      Serial.print((int) i);
      Serial.print("\t0x");
      Serial.println((int) Data[i], HEX);
    }
  #endif

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
