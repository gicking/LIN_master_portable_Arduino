/**
  \file     LIN_master_SoftwareSerial.cpp
  \brief    LIN master emulation library for SoftwareSerial
  \details  This library provides a master node emulation for a LIN bus via SoftwareSerial.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

// assert platform which supports SoftwareSerial. Note: ARDUINO_ARCH_ESP32 works, but requires library ESPSoftwareSerial
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_ESP8266) // || defined(ARDUINO_ARCH_ESP32) 

// include files
#include "Arduino.h"
#include "LIN_master_SoftwareSerial.h"


/**
  \brief      Send LIN break
  \details    Send LIN break (=16bit low). Here blocking!
  \return     current state of LIN state machine
*/
LIN_Master::state_t LIN_Master_SoftwareSerial::_sendBreak(void)
{
  // print debug message
  #if defined(LIN_DEBUG_SERIAL) && (LIN_DEBUG_LEVEL >= 2)
    LIN_DEBUG_SERIAL.println("LIN_Master_SoftwareSerial::_sendBreak()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master::STATE_IDLE)
  {
    this->error = (LIN_Master::error_t) ((int) this->error | (int) LIN_Master::ERROR_STATE);
    this->state = LIN_Master::STATE_DONE;
    return this->state;
  }

  // generate BREAK directly via GPIO (less overhead)
  digitalWrite(pinTx, LOW);
  delayMicroseconds(durationBreak);      
  digitalWrite(pinTx, HIGH);
  delayMicroseconds(100);           // stop bit + 1b delimiter

  // progress state
  this->state = LIN_Master::STATE_BREAK;

  // return state
  return this->state;

} // LIN_Master_SoftwareSerial::_sendBreak()



/**
  \brief      Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \details    Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID). Here blocking!
  \return     current state of LIN state machine
*/
LIN_Master::state_t LIN_Master_SoftwareSerial::_sendFrame(void)
{
  // print debug message
  #if defined(LIN_DEBUG_SERIAL) && (LIN_DEBUG_LEVEL >= 2)
    LIN_DEBUG_SERIAL.println("LIN_Master_SoftwareSerial::_sendFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master::STATE_BREAK)
  {
    this->error = (LIN_Master::error_t) ((int) this->error | (int) LIN_Master::ERROR_STATE);
    this->state = LIN_Master::STATE_DONE;
    return this->state;
  }

  // restore nominal baudrate not required, use dingitalWrite() for BREAK

  // disable reception to skip LIN echo (disturbs SoftwareSerial)
  this->pSerial->stopListening();
  
  // send rest of frame (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID). Is blocking and receive is disabled!
  this->pSerial->write(this->bufTx+1, this->lenTx-1);

  // re-enable reception (above write is blocking)
  this->pSerial->listen();

  // Emulate LIN echo for sent bytes
  memcpy(this->bufRx, this->bufTx, this->lenTx);

  // progress state
  this->state = LIN_Master::STATE_BODY;
  
  // return state
  return this->state;

} // LIN_Master_SoftwareSerial::_sendFrame()



/**
  \brief      Receive and check LIN frame
  \details    Receive and check LIN frame (request frame: check echo; response frame: check header echo & checksum). Here dummy!
  \return     current state of LIN state machine
*/
LIN_Master::state_t LIN_Master_SoftwareSerial::_receiveFrame(void)
{
  // print debug message
  #if defined(LIN_DEBUG_SERIAL) && (LIN_DEBUG_LEVEL >= 2)
    LIN_DEBUG_SERIAL.println("LIN_Master_SoftwareSerial::_receiveFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master::STATE_BODY)
  {
    this->error = (LIN_Master::error_t) ((int) this->error | (int) LIN_Master::ERROR_STATE);
    this->state = LIN_Master::STATE_DONE;
    return this->state;
  }

  // master request frame
  if (this-> type == LIN_Master::MASTER_REQUEST)
  {
    // LIN echo already emulated in _sendFrame()
    
    // check frame for errors
    this->error = (LIN_Master::error_t) ((int) this->error | (int) this->_checkFrame());

    // progress state
    this->state = LIN_Master::STATE_DONE;

  } // master request frame
  
  // slave response frame
  else
  {
    // slave response received (-lenTx because header already emulated in _sendFrame())
    if (this->pSerial->available() >= this->lenRx - this->lenTx)
    {
      // store bytes in Rx
      this->pSerial->readBytes(this->bufRx+3, this->lenRx - this->lenTx);

      // check frame for errors
      this->error = (LIN_Master::error_t) ((int) this->error | (int) this->_checkFrame());

      // progress state
      this->state = LIN_Master::STATE_DONE;

    } // frame body received
  
    // frame body received not yet received
    else
    {
      // check for timeout
      if (micros() - this->timeStart > this->timeMax)
      {
        this->error = (LIN_Master::error_t) ((int) this->error | (int) LIN_Master::ERROR_TIMEOUT);
        this->state = LIN_Master::STATE_DONE;
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
  \param[in]  InverseLogic  use inverse logic
  \param[in]  NameLIN       LIN node name 
*/
LIN_Master_SoftwareSerial::LIN_Master_SoftwareSerial(uint8_t PinRx, uint8_t PinTx, bool InverseLogic, const char NameLIN[]) : LIN_Master::LIN_Master(NameLIN)
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
  \details    Open serial interface with specified baudrate. Here dummy!
  \param[in]  Baudrate    communication speed [Baud]
*/
void LIN_Master_SoftwareSerial::begin(uint16_t Baudrate)
{
  // call base class method
  LIN_Master::begin(Baudrate);
  
  // calculate duration of BREAK
  this->durationBreak = this->timePerByte * 13 / 10;
  
  // open serial interface
  this->pSerial->begin(this->baudrate);
  while(!(*(this->pSerial)));

} // LIN_Master_SoftwareSerial::begin()



/**
  \brief      Close serial interface
  \details    Close serial interface. Here dummy!
*/
void LIN_Master_SoftwareSerial::end()
{
  // call base class method
  LIN_Master::end();
    
  // close serial interface
  this->pSerial->end();

} // LIN_Master_SoftwareSerial::end()

#endif // ARDUINO_ARCH_AVR || ARDUINO_ARCH_ESP8266 || ARDUINO_ARCH_ESP32

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/