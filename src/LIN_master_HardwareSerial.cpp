/**
  \file     LIN_master_HardwareSerial.cpp
  \brief    LIN master emulation library using a HardwareSerial interface
  \details  This library provides a master node emulation for a LIN bus via a HardwareSerial interface, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

// include files
#include <LIN_master_HardwareSerial.h>


/**
  \brief      Send LIN break
  \details    Send LIN break (=16bit low)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial::_sendBreak(void)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial::_sendBreak()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_IDLE)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    _disableTransmitter();
    return this->state;
  }

  // empty buffers, just in case...
  this->pSerial->flush();
  while (this->pSerial->available())
    this->pSerial->read();

  // set half baudrate for BREAK
  this->pSerial->begin(this->baudrate >> 1);
  while(!(*(this->pSerial)));

  // optionally enable transmitter
  _enableTransmitter();

  // send BREAK (>=13 bit low)
  this->pSerial->write(bufTx[0]);

  // progress state
  this->state = LIN_Master_Base::STATE_BREAK;

  // return state
  return this->state;

} // LIN_Master_HardwareSerial::_sendBreak()



/**
  \brief      Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \details    Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial::_sendFrame(void)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial::_sendFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BREAK)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    _disableTransmitter();
    return this->state;
  }

  // byte(s) received (likely BREAK echo)
  if (this->pSerial->available())
  {
    // store echo in Rx
    this->bufRx[0] = this->pSerial->read();

    // restore nominal baudrate
    this->pSerial->begin(this->baudrate);
    while(!(*(this->pSerial)));

    // send rest of frame (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
    this->pSerial->write(this->bufTx+1, this->lenTx-1);

    // progress state
    this->state = LIN_Master_Base::STATE_BODY;

  } // BREAK echo received
  
  // no byte(s) received
  else
  {
    // check for timeout
    if (micros() - this->timeStart > this->timeoutFrame)
    {
      this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_TIMEOUT);
      this->state = LIN_Master_Base::STATE_DONE;
      _disableTransmitter();
    }

  } // no byte(s) received
  
  // return state
  return this->state;

} // LIN_Master_HardwareSerial::_sendFrame()



/**
  \brief      Receive and check LIN frame
  \details    Receive and check LIN frame (request frame: check echo; response frame: check header echo & checksum)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial::_receiveFrame(void)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial::_receiveFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BODY)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    _disableTransmitter();
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
    if (micros() - this->timeStart > this->timeoutFrame)
    {
      this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_TIMEOUT);
      this->state = LIN_Master_Base::STATE_DONE;
      _disableTransmitter();
    }

  } // not enough bytes received
  
  // return state
  return this->state;

} // LIN_Master_HardwareSerial::_receiveFrame()


/**
  \brief      Constructor for LIN node class using HardwareSerial
  \details    Constructor for LIN node class for using HardwareSerial. Store pointer to used serial interface.
  \param[in]  Interface   serial interface for LIN
  \param[in]  NameLIN     LIN node name (default = "Master")
  \param[in]  PinTxEN     optional Tx enable pin (high active) e.g. for LIN via RS485 (default = -127/none)
*/
LIN_Master_HardwareSerial::LIN_Master_HardwareSerial(HardwareSerial &Interface, const char NameLIN[], const int8_t PinTxEN) : 
  LIN_Master_Base::LIN_Master_Base(NameLIN, PinTxEN)
{
  // Debug serial initialized in begin() -> no debug output here

  // store pointer to used HW serial
  this->pSerial = &Interface;

  // must not open connection here, else (at least) ESP32 and ESP8266 fail

} // LIN_Master_HardwareSerial::LIN_Master_HardwareSerial()



/**
  \brief      Open serial interface
  \details    Open serial interface with specified baudrate
  \param[in]  Baudrate    communication speed [Baud] (default = 19200)
*/
void LIN_Master_HardwareSerial::begin(uint16_t Baudrate)
{  
  // call base class method
  LIN_Master_Base::begin(Baudrate);

  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.print(": LIN_Master_HardwareSerial::begin(");
    LIN_MASTER_DEBUG_SERIAL.print((int) Baudrate);
    LIN_MASTER_DEBUG_SERIAL.println(")");
  #endif

  // open serial interface
  this->pSerial->begin(this->baudrate);
  while(!(*(this->pSerial)));

} // LIN_Master_HardwareSerial::begin()



/**
  \brief      Close serial interface
  \details    Close serial interface
*/
void LIN_Master_HardwareSerial::end()
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.print(": LIN_Master_HardwareSerial::end()");
  #endif

  // call base class method
  LIN_Master_Base::end();
    
  // close serial interface
  this->pSerial->end();

} // LIN_Master_HardwareSerial::end()

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
