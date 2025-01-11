/**
  \file     LIN_master_HardwareSerial_ESP8266.cpp
  \brief    LIN master emulation library using hardware Serial0 interface of ESP8266
  \details  This library provides a master node emulation for a LIN bus via hardware Serial0 interface of ESP8266, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \note     Serial.begin() causes a glitch on the bus. Therefore use Serial.updateBaudRate() instead.
  \note     Serial.flush() is omitted because it causes a 500us delay, see https://github.com/esp8266/Arduino/blob/master/cores/esp8266/HardwareSerial.cpp
  \author   Georg Icking-Konert
*/

// assert ESP8266 platform
#if defined(ARDUINO_ARCH_ESP8266)

// include files
#include <LIN_master_HardwareSerial_ESP8266.h>


/**
  \brief      Send LIN break
  \details    Send LIN break (=16bit low)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial_ESP8266::_sendBreak(void)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial_ESP8266::_sendBreak()");
  #endif
  
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_IDLE)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    return this->state;
  }

  // empty buffers, just in case...
  //this->pSerial->flush();   // skip, as this causes a ~500us delay, see https://github.com/esp8266/Arduino/blob/master/cores/esp8266/HardwareSerial.cpp
  while (this->pSerial->available())
    this->pSerial->read();

  // set half baudrate for BREAK
  this->pSerial->updateBaudRate(this->baudrate >> 1);

  // optionally enable transmitter
  _enableTransmitter();

  // send BREAK (>=13 bit low)
  this->pSerial->write(bufTx[0]);

  // progress state
  this->state = LIN_Master_Base::STATE_BREAK;

  // return state
  return this->state;

} // LIN_Master_HardwareSerial_ESP8266::_sendBreak()



/**
  \brief      Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \details    Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial_ESP8266::_sendFrame(void)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial_ESP8266::_sendFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BREAK)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    return this->state;
  }

  // byte(s) received (likely BREAK echo)
  if (this->pSerial->available())
  {
    // store echo in Rx
    this->bufRx[0] = this->pSerial->read();

    // restore nominal baudrate
    this->pSerial->updateBaudRate(this->baudrate);

    // send rest of frame (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
    this->pSerial->write(this->bufTx+1, this->lenTx-1);

    // progress state
    this->state = LIN_Master_Base::STATE_BODY;

  } // BREAK echo received
  
  // no byte(s) received
  else
  {
    // check for timeout
    if (micros() - this->timeStart > this->timeMax)
    {
      this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_TIMEOUT);
      _disableTransmitter();
      this->state = LIN_Master_Base::STATE_DONE;
    }

  } // no byte(s) received
  
  // return state
  return this->state;

} // LIN_Master_HardwareSerial_ESP8266::_sendFrame()



/**
  \brief      Receive and check LIN frame
  \details    Receive and check LIN frame (request frame: check echo; response frame: check header echo & checksum)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial_ESP8266::_receiveFrame(void)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial_ESP8266::_receiveFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BODY)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    return this->state;
  }

  // optionally disable transmitter for slave response frames. Len==2 because BREAK is handled already handled in _sendFrame()
  if ((this->type == LIN_Master_Base::SLAVE_RESPONSE) && (this->pSerial->available() == 2))
    _disableTransmitter();

  // frame body received (-1 because BREAK is handled already handled in _sendFrame())
  if (this->pSerial->available() >= this->lenRx-1)
  {
    // store bytes in Rx
    this->pSerial->readBytes(this->bufRx+1, this->lenRx-1);

    // check frame for errors
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) this->_checkFrame());

    // optionally disable transmitter after frame is completed
    _disableTransmitter();

    // progress state
    this->state = LIN_Master_Base::STATE_DONE;

  } // frame body received
  
  // frame body received not yet received
  else
  {
    // check for timeout
    if (micros() - this->timeStart > this->timeMax)
    {
      this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_TIMEOUT);
      _disableTransmitter();
      this->state = LIN_Master_Base::STATE_DONE;
    }

  } // not enough bytes received
  
  // return state
  return this->state;

} // LIN_Master_HardwareSerial_ESP8266::_receiveFrame()



/**
  \brief      Constructor for LIN node class using ESP8266 HardwareSerial 0
  \details    Constructor for LIN node class for using ESP8266 HardwareSerial 0.
  \param[in]  SwapPins    use alternate Serial2 Rx/Tx pins (default = false)
  \param[in]  NameLIN     LIN node name (default = "Master")
  \param[in]  PinTxEN     optional Tx enable pin (high active) e.g. for LIN via RS485 (default = -127/none)
*/
LIN_Master_HardwareSerial_ESP8266::LIN_Master_HardwareSerial_ESP8266(bool SwapPins, const char NameLIN[], const int8_t PinTxEN) : 
  LIN_Master_Base::LIN_Master_Base(NameLIN, PinTxEN)
{
  // print debug message (debug level 2)
  // Note: not be printed, because constructor is called prior to setup()
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial_ESP8266::LIN_Master_HardwareSerial_ESP8266()");
  #endif

  // store pointer to used HW serial
  this->pSerial    = &Serial;                               // ESP8266 only has 1 UART with send/receive
  this->swapPins   = SwapPins;                              // use alternate pins Rx=D7 / Tx=D8 for Serial0

  // must not open connection here, else system resets

} // LIN_Master_HardwareSerial_ESP8266::LIN_Master_HardwareSerial_ESP8266()



/**
  \brief      Open serial interface
  \details    Open serial interface with specified baudrate. Optionally use Serial2 pins 
  \param[in]  Baudrate    communication speed [Baud] (default = 19200)
*/
void LIN_Master_HardwareSerial_ESP8266::begin(uint16_t Baudrate)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.print(": LIN_Master_HardwareSerial_ESP8266::begin(");
    LIN_MASTER_DEBUG_SERIAL.print((int) Baudrate);
    LIN_MASTER_DEBUG_SERIAL.println(")");
  #endif

  // call base class method
  LIN_Master_Base::begin(Baudrate);

  // open serial interface
  this->pSerial->begin(this->baudrate, SERIAL_8N1);
  while(!(*(this->pSerial)));

  // optionally route Serial0 to alternate pins
  if (this->swapPins == true)
    this->pSerial->swap();

} // LIN_Master_HardwareSerial_ESP8266::begin()



/**
  \brief      Close serial interface
  \details    Close serial interface
*/
void LIN_Master_HardwareSerial_ESP8266::end()
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.print(": LIN_Master_HardwareSerial_ESP8266::end()");
  #endif

  // call base class method
  LIN_Master_Base::end();
    
  // close serial interface
  this->pSerial->end();

} // LIN_Master_HardwareSerial_ESP8266::end()

#endif // ARDUINO_ARCH_ESP8266

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
