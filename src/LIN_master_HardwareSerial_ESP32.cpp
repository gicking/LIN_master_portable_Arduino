/**
  \file     LIN_master_HardwareSerial_ESP32.cpp
  \brief    LIN master emulation library using a HardwareSerial interface of ESP32
  \details  This library provides a master node emulation for a LIN bus via a HardwareSerial interface of ESP32, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \note     Serial.available() has >1ms delay, likely due to 2nd Core implementation, see https://esp32.com/viewtopic.php?p=65158. Use BREAK duration instead
  \author   Georg Icking-Konert
*/

// assert ESP32 platform
#if defined(ARDUINO_ARCH_ESP32)

// include files
#include <LIN_master_HardwareSerial_ESP32.h>


/**
  \brief      Send LIN break
  \details    Send LIN break (=16bit low)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial_ESP32::_sendBreak(void)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial_ESP32::_sendBreak()");
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
  this->pSerial->updateBaudRate(this->baudrate >> 1);

  // optionally enable transmitter
  _enableTransmitter();

  // send BREAK (>=13 bit low)
  this->pSerial->write(bufTx[0]);

  // store starting time to avoid using Serial.available(), which has >1ms delay
  this->timeStartBreak = micros();

  // progress state
  this->state = LIN_Master_Base::STATE_BREAK;

  // return state
  return this->state;

} // LIN_Master_HardwareSerial_ESP32::_sendBreak()



/**
  \brief      Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \details    Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial_ESP32::_sendFrame(void)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial_ESP32::_sendFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BREAK)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    _disableTransmitter();
    return this->state;
  }

  // Serial.available() has >1ms delay -> use duration of BREAK instead
  if ((micros() - this->timeStartBreak) > (timePerByte << 1))
  {
    // skip reading Rx now (is not yet in buffer)

    // restore nominal baudrate. Apparently this is ok for BREAK
    this->pSerial->updateBaudRate(this->baudrate);

    // send rest of frame (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
    this->pSerial->write(this->bufTx+1, this->lenTx-1);

    // progress state
    this->state = LIN_Master_Base::STATE_BODY;

  } // BREAK duration expired
    
  // return state
  return this->state;

} // LIN_Master_HardwareSerial_ESP32::_sendFrame()



/**
  \brief      Receive and check LIN frame
  \details    Receive and check LIN frame (request frame: check echo; response frame: check header echo & checksum)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial_ESP32::_receiveFrame(void)
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.println(": LIN_Master_HardwareSerial_ESP32::_receiveFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BODY)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    _disableTransmitter();
    return this->state;
  }

  // optionally disable transmitter for slave response frames. Here, need to read BREAK as well due to delay of Serial.available()
  if ((this->type == LIN_Master_Base::SLAVE_RESPONSE) && (this->pSerial->available() == 3))
    _disableTransmitter();

  // frame body received. Here, need to read BREAK as well due to delay of Serial.available()
  if (this->pSerial->available() >= this->lenRx)
  {
    // store bytes in Rx
    this->pSerial->readBytes(this->bufRx, this->lenRx);

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

} // LIN_Master_HardwareSerial_ESP32::_receiveFrame()



/**
  \brief      Constructor for LIN node class using ESP32 HardwareSerial
  \details    Constructor for LIN node class for using ESP32 HardwareSerial. Store pointer to used serial interface w/ pins.
  \param[in]  Interface   serial interface for LIN
  \param[in]  PinRx       GPIO used for reception
  \param[in]  PinTx       GPIO used for transmission
  \param[in]  NameLIN     LIN node name (default = "Master")
  \param[in]  PinTxEN     optional Tx enable pin (high active) e.g. for LIN via RS485 (default = -127/none)
*/
LIN_Master_HardwareSerial_ESP32::LIN_Master_HardwareSerial_ESP32(HardwareSerial &Interface, uint8_t PinRx, uint8_t PinTx, const char NameLIN[], const int8_t PinTxEN) :
  LIN_Master_Base::LIN_Master_Base(NameLIN, PinTxEN)
{
  // Debug serial initialized in begin() -> no debug output here

  // store pointer to used HW serial
  this->pSerial    = &Interface;                              // used serial interface
  this->pinRx      = PinRx;                                   // receive pin
  this->pinTx      = PinTx;                                   // transmit pin

  // must not open connection here, else system resets

} // LIN_Master_HardwareSerial_ESP32::LIN_Master_HardwareSerial_ESP32()



/**
  \brief      Open serial interface
  \details    Open serial interface with specified baudrate
  \param[in]  Baudrate    communication speed [Baud] (default = 19200)
*/
void LIN_Master_HardwareSerial_ESP32::begin(uint16_t Baudrate)
{
  // call base class method
  LIN_Master_Base::begin(Baudrate);

  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.print(": LIN_Master_HardwareSerial_ESP32::begin(");
    LIN_MASTER_DEBUG_SERIAL.print((int) Baudrate);
    LIN_MASTER_DEBUG_SERIAL.println(")");
  #endif

  // open serial interface incl. used pins
  this->pSerial->end();
  this->pSerial->begin(baudrate, SERIAL_8N1, pinRx, pinTx);
  while(!(*(this->pSerial)));

} // LIN_Master_HardwareSerial_ESP32::begin()



/**
  \brief      Close serial interface
  \details    Close serial interface
*/
void LIN_Master_HardwareSerial_ESP32::end()
{
  // print debug message (debug level 2)
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.print(this->nameLIN);
    LIN_MASTER_DEBUG_SERIAL.print(": LIN_Master_HardwareSerial_ESP32::end()");
  #endif

  // call base class method
  LIN_Master_Base::end();
    
  // close serial interface
  this->pSerial->end();

} // LIN_Master_HardwareSerial_ESP32::end()

#endif // ARDUINO_ARCH_ESP32

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
