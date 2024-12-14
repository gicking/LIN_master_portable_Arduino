/**
  \file     LIN_master_SoftwareSerial.cpp
  \brief    LIN master emulation library for SoftwareSerial
  \details  This library provides a master node emulation for a LIN bus via SoftwareSerial, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

// assert platform which supports SoftwareSerial. Note: ARDUINO_ARCH_ESP32 requires library ESPSoftwareSerial
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) 

// include files
#include <LIN_master_SoftwareSerial.h>


/**
  \brief      Send LIN break
  \details    Send LIN break (=16bit low). Here blocking!
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_SoftwareSerial::_sendBreak(void)
{
  // print debug message
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_SoftwareSerial::_sendBreak()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_IDLE)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    return this->state;
  }

  // optionally enable transmitter
  enableTransmitter();

  // generate BREAK directly via GPIO (less overhead)
  digitalWrite(pinTx, LOW);
  delayMicroseconds(durationBreak);      
  digitalWrite(pinTx, HIGH);
  delayMicroseconds(100);           // stop bit + 1b delimiter

  // progress state
  this->state = LIN_Master_Base::STATE_BREAK;

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
  // print debug message
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_SoftwareSerial::_sendFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BREAK)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    return this->state;
  }

  // restore nominal baudrate not required, use digitalWrite() for BREAK

  // disable reception to skip LIN echo (disturbs SoftwareSerial)
  ((SoftwareSerial*) this->pSerial)->stopListening();
  
  // send rest of frame (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID). Is blocking and receive is disabled!
  ((SoftwareSerial*) this->pSerial)->write(this->bufTx+1, this->lenTx-1);

  // optionally disable transmitter for slave response frames
  if (this->type == LIN_Master_Base::SLAVE_RESPONSE)
    disableTransmitter();

  // re-enable reception (above write is blocking)
  ((SoftwareSerial*) this->pSerial)->listen();

  // Emulate LIN echo for sent bytes
  memcpy(this->bufRx, this->bufTx, this->lenTx);

  // progress state
  this->state = LIN_Master_Base::STATE_BODY;
  
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
  // print debug message
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_SoftwareSerial::_receiveFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master_Base::STATE_BODY)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    return this->state;
  }

  // master request frame
  if (this-> type == LIN_Master_Base::MASTER_REQUEST)
  {
    // LIN echo already emulated in _sendFrame()
    
    // check frame for errors
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) this->_checkFrame());

    // optionally disable transmitter after frame is completed
    disableTransmitter();
    
    // progress state
    this->state = LIN_Master_Base::STATE_DONE;

  } // master request frame
  
  // slave response frame
  else
  {
    // slave response received (-lenTx because header already emulated in _sendFrame())
    if (((SoftwareSerial*) this->pSerial)->available() >= this->lenRx - this->lenTx)
    {
      // store bytes in Rx
      ((SoftwareSerial*) this->pSerial)->readBytes(this->bufRx+3, this->lenRx - this->lenTx);

      // check frame for errors
      this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) this->_checkFrame());

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
        disableTransmitter();
        this->state = LIN_Master_Base::STATE_DONE;
      }

    } // not enough bytes received

  } // slave response frame
  
  // return state
  return this->state;

} // LIN_Master_SoftwareSerial::_receiveFrame()



/**
  \brief      Constructor for LIN node class using SoftwareSerial
  \details    Constructor for LIN node class for using SoftwareSerial. Store pointers to SW serial instance.
  \param[in]  PinRx         GPIO used for reception
  \param[in]  PinTx         GPIO used for transmission
  \param[in]  InverseLogic  use inverse logic (default = false)
  \param[in]  NameLIN       LIN node name (default = "Master")
  \param[in]  PinTxEN       optional Tx enable pin (high active) e.g. for LIN via RS485 (default = -127/none)
*/
LIN_Master_SoftwareSerial::LIN_Master_SoftwareSerial(uint8_t PinRx, uint8_t PinTx, bool InverseLogic, const char NameLIN[], const int8_t PinTxEN) : 
  LIN_Master_Base::LIN_Master_Base(NameLIN, PinTxEN)
{
  // store pins used for SW serial
  this->pinRx = PinRx;
  this->pinTx = PinTx;
  this->inverseLogic = InverseLogic;

  // create new SW serial
  pSerial = new SoftwareSerial(this->pinRx, this->pinTx, this->inverseLogic);
  
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
  
  // calculate duration of BREAK
  this->durationBreak = this->timePerByte * 13 / 10;
  
  // open serial interface
  ((SoftwareSerial*) this->pSerial)->begin(this->baudrate);
  while(!(*(((SoftwareSerial*) this->pSerial))));

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
  ((SoftwareSerial*) this->pSerial)->end();

} // LIN_Master_SoftwareSerial::end()


#endif // ARDUINO_ARCH_AVR || ARDUINO_ARCH_ESP8266 || ARDUINO_ARCH_ESP32

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
