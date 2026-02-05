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
LIN_Master_Base::state_t LIN_master_SoftwareSerial::_sendBreak(void)
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

  // Renesas core does not support listen()/stopListening(), see https://github.com/arduino/ArduinoCore-renesas/issues/522
  #if !defined(ARDUINO_ARCH_RENESAS)
    this->SWSerial.stopListening();
  #endif

  // optionally enable transmitter
  this->_enableTransmitter();

  // start BREAK directly via GPIO (less overhead than begin()). End BREAK in _sendFrame()
  // Also bug in Renesas core: https://github.com/arduino/ArduinoCore-renesas/issues/523
  digitalWrite(this->pinTx, this->inverseLogic ? HIGH : LOW);
  
  // store BREAK start time
  this->startBreak = micros();

  // progress state
  this->state = LIN_Master_Base::STATE_BREAK;

  // print debug message
  DEBUG_PRINT(3, " ");

  // return state
  return this->state;

} // LIN_master_SoftwareSerial::_sendBreak()



/**
  \brief      Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \details    Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID). Here blocking!
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_master_SoftwareSerial::_sendFrame(void)
{
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

  // after BREAK duration elapsed, release GPIO again and check for LIN echo
  if ((micros() - this->startBreak) > this->durationBreak)
  {
    // terminate BREAK
    digitalWrite(this->pinTx, this->inverseLogic ? LOW : HIGH);

    // assert >=1b BREAK delimiter
    delayMicroseconds(50);
        
    // For STM32, listen must be before write
    #if defined(ARDUINO_ARCH_STM32)
      this->SWSerial.listen(); 
    #endif

    // blocking send rest of frame (response: SYNC+ID; request: SYNC+ID+DATA[]+CHK)
    this->SWSerial.write(this->bufTx+1, this->lenTx-1);
    //this->SWSerial.flush();   // not required because write() is blocking. Causes issue on STM32

    // optionally disable RS485 transmitter after frame is completed
    this->_disableTransmitter();

    // Renesas core does not support listen()/stopListening(), see https://github.com/arduino/ArduinoCore-renesas/issues/522
    // For STM32, listen must be before write above
    #if !defined(ARDUINO_ARCH_RENESAS) && !defined(ARDUINO_ARCH_STM32)
      this->SWSerial.listen(); 
    #endif
    
    // progress state
    this->state = LIN_Master_Base::STATE_BODY;
  
  } // after BREAK elapsed

  // print debug message
  DEBUG_PRINT(2, " ");
    
  // return state
  return this->state;

} // LIN_master_SoftwareSerial::_sendFrame()



/**
  \brief      Receive and check LIN frame
  \details    Receive and check LIN frame (request frame: check echo; response frame: check header echo & checksum)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_master_SoftwareSerial::_receiveFrame(void)
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

// Renesas core does not support listen()/stopListening() --> receive echo & response
#if defined(ARDUINO_ARCH_RENESAS)

  // echo + frame body received
  if (this->SWSerial.available() >= this->lenRx)
  {  
    // read into buffer
    this->SWSerial.readBytes(this->bufRx, this->lenRx);

// STM32 core receives echo w/o BREAK & response
#elif defined(ARDUINO_ARCH_STM32)

  // echo (w/o BREAK) + frame body received
  if (this->SWSerial.available() >= this->lenRx-1)
  {  
    // read into buffer
    this->SWSerial.readBytes(this->bufRx+1, this->lenRx-1);

    // emulate only ignored BREAK
    this->bufRx[0] = this->bufTx[0];

// other cores only receive slave response
#else

  // only response received
  if (this->SWSerial.available() >= this->lenRx - this->lenTx) {
      
    // read into buffer
    this->SWSerial.readBytes(this->bufRx + this->lenTx, this->lenRx - this->lenTx);

    // emulate ignored echo for sent bytes (incl. BREAK)
    memcpy(this->bufRx, this->bufTx, this->lenTx);

#endif // ARDUINO_ARCH_RENESAS

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
      DEBUG_PRINT(1, "Rx timeout");

      // set error state and return immediately
      this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_TIMEOUT);
      this->state = LIN_Master_Base::STATE_DONE;
      this->_disableTransmitter();
      return this->state;
    }

  } // not enough bytes received
  
  // print debug message
  DEBUG_PRINT(2, " ");
  
  // return state
  return this->state;

} // LIN_master_SoftwareSerial::_receiveFrame()



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
  LIN_master_SoftwareSerial::LIN_master_SoftwareSerial(uint8_t PinRx, uint8_t PinTx, bool InverseLogic, const char NameLIN[], const int8_t PinTxEN) : 
    LIN_Master_Base::LIN_Master_Base(NameLIN, PinTxEN), SWSerial(PinRx, PinTx)
#else
  LIN_master_SoftwareSerial::LIN_master_SoftwareSerial(uint8_t PinRx, uint8_t PinTx, bool InverseLogic, const char NameLIN[], const int8_t PinTxEN) : 
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

} // LIN_master_SoftwareSerial::LIN_master_SoftwareSerial()



/**
  \brief      Open serial interface
  \details    Open serial interface with specified baudrate
  \param[in]  Baudrate    communication speed [Baud] (default = 19200)
*/
void LIN_master_SoftwareSerial::begin(uint16_t Baudrate)
{
  // call base class method
  LIN_Master_Base::begin(Baudrate);
  
  // open serial interface. Timeout not required here
  this->SWSerial.end();
  #if defined(ARDUINO_ARCH_RENESAS)
    this->SWSerial.begin(this->baudrate, SERIAL_8N1, this->inverseLogic);
  #else
    this->SWSerial.begin(this->baudrate);
  #endif

  // calculate duration of BREAK
  this->durationBreak = this->timePerByte * 16 / 10;
 
  // print debug message
  DEBUG_PRINT(2, "ok");

} // LIN_master_SoftwareSerial::begin()



/**
  \brief      Close serial interface
  \details    Close serial interface
*/
void LIN_master_SoftwareSerial::end()
{
  // call base class method
  LIN_Master_Base::end();
    
  // close serial interface
  this->SWSerial.end();

  // print debug message
  DEBUG_PRINT(2, " ");

} // LIN_master_SoftwareSerial::end()


#endif // ARDUINO_ARCH_AVR || ARDUINO_ARCH_ESP8266 || ARDUINO_ARCH_ESP32 || ARDUINO_ARCH_MEGAAVR || ARDUINO_ARCH_STM32 || ARDUINO_ARCH_RENESAS


/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
