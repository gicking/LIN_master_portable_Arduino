/**
  \file     LIN_master_SoftwareSerial.cpp
  \brief    LIN master emulation library for SoftwareSerial
  \details  This library provides a master node emulation for a LIN bus via blocking SoftwareSerial, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

// assert platform which supports SoftwareSerial. Note: ARDUINO_ARCH_ESP32 requires library ESPSoftwareSerial
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || \
  defined(ARDUINO_ARCH_MEGAAVR) || defined(ARDUINO_ARCH_STM32) || defined(ARDUINO_ARCH_RENESAS)

// include files
#include <LIN_master_SoftwareSerial.h>


/**
  \brief      Send LIN break
  \details    Send LIN break (=16bit low). Here blocking!
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_SoftwareSerial::_sendBreak(void)
{
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_IDLE)
  {
    // print debug message
    DEBUG_PRINT(1, "wrong state 0x%02X", this->state);

    // set error state and return immediately
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    this->_disableTransmitter();
    return this->state;
  }

  // empty buffers, just in case...
  this->SWSerial.flush();
  while (this->SWSerial.available())
    this->SWSerial.read();

  // optionally enable transmitter
  this->_enableTransmitter();

  // for Renesas core temporarily close SW serial. Notes:
  //   - using SWSerial to send BREAK doesn't work, see https://github.com/arduino/ArduinoCore-renesas/issues/523
  //   - listen()/stopListening() not yet implemented, see https://github.com/arduino/ArduinoCore-renesas/issues/522
  #if defined(ARDUINO_ARCH_RENESAS)
    this->SWSerial.end();
    pinMode(this->pinTx, OUTPUT);

  // for other cores only disable reception
  #else
    this->SWSerial.stopListening();
  #endif

  // generate BREAK directly via GPIO (less overhead than begin())
  digitalWrite(this->pinTx, this->inverseLogic ? HIGH : LOW);
  delayMicroseconds(this->durationBreak);      
  digitalWrite(this->pinTx, this->inverseLogic ? LOW : HIGH);
  delayMicroseconds(100);           // stop bit + 1b delimiter

  // progress state
  this->state = LIN_Master_Base::STATE_BREAK;

  // print debug message
  DEBUG_PRINT(3, " ");

  // return state
  return this->state;

} // LIN_Master_SoftwareSerial::_sendBreak()



/**
  \brief      Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \details    Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID). Here blocking!
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_SoftwareSerial::_sendFrame(void)
{    
  // temp buffer for echo bytes (max: SYNC+ID+DATA[8]+CHK)
  #if defined(ARDUINO_ARCH_RENESAS) || defined(ARDUINO_ARCH_STM32)
    uint8_t   bufEcho[11];
  #endif

  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BREAK)
  {
    // print debug message
    DEBUG_PRINT(1, "wrong state 0x%02X", this->state);

    // set error state and return immediately
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    this->_disableTransmitter();
    return this->state;
  }

  // restore nominal baudrate not required, used digitalWrite() for BREAK above

  // for Renesas core re-open SW serial & flush Rx buffer. Notes:
  //   - listen()/stopListening() not yet implemented, see https://github.com/arduino/ArduinoCore-renesas/issues/522
  #if defined(ARDUINO_ARCH_RENESAS)
    this->SWSerial.begin(this->baudrate, this->inverseLogic);
    while (this->SWSerial.available())
      this->SWSerial.read();
  #endif

  // for STM32 core re-enable reception. Apparently listen() takes long to take effect, so we call it before write()
  #if defined (ARDUINO_ARCH_STM32)
    this->SWSerial.listen();
  #endif

  // send rest of frame (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID). Is blocking and receive is disabled!
  this->SWSerial.write(this->bufTx+1, this->lenTx-1);
  this->SWSerial.flush();

  // optionally disable RS485 transmitter for slave response frames
  if (this->type == LIN_Master_Base::SLAVE_RESPONSE)
    this->_disableTransmitter();

  // for Renesas core flush Rx buffer. Notes:
  //   - listen()/stopListening() not yet implemented, see https://github.com/arduino/ArduinoCore-renesas/issues/522
  #if defined(ARDUINO_ARCH_RENESAS)
    delayMicroseconds(10);          // wait a bit to ensure LIN echo is in Rx buffer
    this->SWSerial.readBytes(bufEcho, min((int) (this->SWSerial.available()), (int) (this->lenTx-1)));

  // for STM32 core flush Rx buffer
  #elif defined(ARDUINO_ARCH_STM32) 
    delayMicroseconds(10);          // wait a bit to ensure LIN echo is in Rx buffer
    this->SWSerial.readBytes(bufEcho, min((int) (this->SWSerial.available()), (int) (this->lenTx-1)));

  // for other cores re-enable reception
  #else
    this->SWSerial.listen();
  #endif

  // Emulate LIN echo for sent bytes (w/o BREAK)
  memcpy(this->bufRx, this->bufTx, this->lenTx);

  // progress state
  this->state = LIN_Master_Base::STATE_BODY;
  
  // print debug message
  DEBUG_PRINT(2, " ");
    
  // return state
  return this->state;

} // LIN_Master_SoftwareSerial::_sendFrame()



/**
  \brief      Receive and check LIN frame
  \details    Receive and check LIN frame (request frame: check echo; response frame: check header echo & checksum)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_SoftwareSerial::_receiveFrame(void)
{
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BODY)
  {
    // print debug message
    DEBUG_PRINT(1, "wrong state 0x%02X", this->state);

    // set error state and return immediately
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    this->_disableTransmitter();
    return this->state;
  }

  // master request frame
  if (this-> type == LIN_Master_Base::MASTER_REQUEST)
  {
    // LIN echo already emulated in _sendFrame()
    
    // check frame for errors
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) this->_checkFrame());

    // optionally disable RS485 transmitter after frame is completed
    this->_disableTransmitter();
    
    // progress state
    this->state = LIN_Master_Base::STATE_DONE;

  } // master request frame
  
  // slave response frame
  else
  {
    // slave response received (-lenTx because header already emulated in _sendFrame())
    if (this->SWSerial.available() >= this->lenRx - this->lenTx)
    {
      // store bytes in Rx
      this->SWSerial.readBytes(this->bufRx+3, this->lenRx - this->lenTx);

      // check frame for errors
      this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) this->_checkFrame());

      // progress state
      this->state = LIN_Master_Base::STATE_DONE;

    } // frame body received
  
    // frame body received not yet received
    else
    {
      // check for timeout
      if (micros() - this->timeStart > this->timeoutFrame)
      {
        // print debug message
        DEBUG_PRINT(1, "Rx timeout (%dB vs. %dB)", this->SWSerial.available(), this->lenRx - this->lenTx);

        // for debug only
        #if defined(LIN_MASTER_DEBUG_SERIAL)
          int num = this->SWSerial.available();
          for (int i=0; i<num; i++) {
            LIN_MASTER_DEBUG_SERIAL.print(i);
            LIN_MASTER_DEBUG_SERIAL.print(": 0x");
            int c = this->SWSerial.read();
            LIN_MASTER_DEBUG_SERIAL.println(c, HEX);
          }
          LIN_MASTER_DEBUG_SERIAL.println("---");
        #endif

        // set error state and return immediately
        this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_TIMEOUT);
        this->state = LIN_Master_Base::STATE_DONE;
        this->_disableTransmitter();
        return this->state;
      }

    } // not enough bytes received

  } // slave response frame
  
  // print debug message
  DEBUG_PRINT(2, " ");
  
  // return state
  return this->state;

} // LIN_Master_SoftwareSerial::_receiveFrame()



/**
  \brief      Constructor for LIN node class using SoftwareSerial
  \details    Constructor for LIN node class for using SoftwareSerial. Initialize SW serial instance.
  \param[in]  PinRx         GPIO used for reception
  \param[in]  PinTx         GPIO used for transmission
  \param[in]  InverseLogic  use inverse logic (default = false)
  \param[in]  NameLIN       LIN node name (default = "Master")
  \param[in]  PinTxEN       optional Tx enable pin (high active) e.g. for LIN via RS485 (default = -127/none)
*/
#if defined(ARDUINO_ARCH_RENESAS)   // for Renesas core inverse logic is parameter for begin()
  LIN_Master_SoftwareSerial::LIN_Master_SoftwareSerial(uint8_t PinRx, uint8_t PinTx, bool InverseLogic, const char NameLIN[], const int8_t PinTxEN) : 
    LIN_Master_Base::LIN_Master_Base(NameLIN, PinTxEN), SWSerial(PinRx, PinTx)
#else
  LIN_Master_SoftwareSerial::LIN_Master_SoftwareSerial(uint8_t PinRx, uint8_t PinTx, bool InverseLogic, const char NameLIN[], const int8_t PinTxEN) : 
    LIN_Master_Base::LIN_Master_Base(NameLIN, PinTxEN), SWSerial(PinRx, PinTx, InverseLogic)
#endif
{
  // Debug serial initialized in begin() -> no debug output here

  // store pins used for SW serial
  this->pinRx = PinRx;
  this->pinTx = PinTx;
  this->inverseLogic = InverseLogic;
  
  // must not open connection here, else (at least) ESP32 and ESP8266 fail

  // cannot print debug message, as constructor is called before setup()

} // LIN_Master_SoftwareSerial::LIN_Master_SoftwareSerial()



/**
  \brief      Open serial interface
  \details    Open serial interface with specified baudrate
  \param[in]  Baudrate    communication speed [Baud] (default = 19200)
*/
void LIN_Master_SoftwareSerial::begin(uint16_t Baudrate)
{
  // call base class method
  LIN_Master_Base::begin(Baudrate);
  
  // open serial interface. Timeout not required here
  this->SWSerial.end();
  #if defined(ARDUINO_ARCH_RENESAS)       // for Renesas core inverse logic is parameter for begin()
    this->SWSerial.begin(this->baudrate, this->inverseLogic);
  #else
    this->SWSerial.begin(this->baudrate);
  #endif

  // calculate duration of BREAK
  this->durationBreak = this->timePerByte * 16 / 10;
 
  // print debug message
  DEBUG_PRINT(2, "ok");

} // LIN_Master_SoftwareSerial::begin()



/**
  \brief      Close serial interface
  \details    Close serial interface
*/
void LIN_Master_SoftwareSerial::end()
{
  // call base class method
  LIN_Master_Base::end();
    
  // close serial interface
  this->SWSerial.end();

  // print debug message
  DEBUG_PRINT(2, " ");

} // LIN_Master_SoftwareSerial::end()


#endif // ARDUINO_ARCH_AVR || ARDUINO_ARCH_ESP8266 || ARDUINO_ARCH_ESP32 || ARDUINO_ARCH_MEGAAVR || ARDUINO_ARCH_STM32 || ARDUINO_ARCH_RENESAS


/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
