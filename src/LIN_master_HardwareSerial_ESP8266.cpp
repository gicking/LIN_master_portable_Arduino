/**
  \file     LIN_master_HardwareSerial_ESP8266.cpp
  \brief    LIN master emulation library using hardware Serial0 interface of ESP8266
  \details  This library provides a master node emulation for a LIN bus via hardware Serial0 interface of ESP8266.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \note     Serial.begin() causes a glitch on the bus. Therefore use Serial.updateBaudRate() instead.
  \note     Serial.flush() is omitted because it causes a 500us delay, see https://github.com/esp8266/Arduino/blob/master/cores/esp8266/HardwareSerial.cpp
  \author   Georg Icking-Konert
*/

// assert ESP8266 platform
#if defined(ARDUINO_ARCH_ESP8266)

// include files
#include "Arduino.h"
#include "LIN_master_HardwareSerial_ESP8266.h"


/**
  \brief      Send LIN break
  \details    Send LIN break (=16bit low)
  \return     current state of LIN state machine
*/
LIN_Master::state_t LIN_Master_HardwareSerial_ESP8266::_sendBreak(void)
{
  // print debug message
  #if defined(LIN_DEBUG_SERIAL) && (LIN_DEBUG_LEVEL >= 2)
    LIN_DEBUG_SERIAL.println("LIN_Master_HardwareSerial::_sendBreak()");
  #endif
  
  // if state is wrong, exit immediately
  if (this->state != LIN_Master::STATE_IDLE)
  {
    this->error = (LIN_Master::error_t) ((int) this->error | (int) LIN_Master::ERROR_STATE);
    this->state = LIN_Master::STATE_DONE;
    return this->state;
  }

  // empty buffers, just in case...
  //this->pSerial->flush();   // skip, as this causes a ~500us delay, see https://github.com/esp8266/Arduino/blob/master/cores/esp8266/HardwareSerial.cpp
  while (this->pSerial->available())
    this->pSerial->read();
 
  // set half baudrate for BREAK
  this->pSerial->updateBaudRate(this->baudrate >> 1);

  // send BREAK (>=13 bit low)
  this->pSerial->write(bufTx[0]);

  // progress state
  this->state = LIN_Master::STATE_BREAK;

  // return state
  return this->state;

} // LIN_Master_HardwareSerial_ESP8266::_sendBreak()



/**
  \brief      Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \details    Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \return     current state of LIN state machine
*/
LIN_Master::state_t LIN_Master_HardwareSerial_ESP8266::_sendFrame(void)
{
  // print debug message
  #if defined(LIN_DEBUG_SERIAL) && (LIN_DEBUG_LEVEL >= 2)
    LIN_DEBUG_SERIAL.println("LIN_Master_HardwareSerial::_sendFrame()");
  #endif
    
  // if state is wrong, exit immediately
  if (this->state != LIN_Master::STATE_BREAK)
  {
    this->error = (LIN_Master::error_t) ((int) this->error | (int) LIN_Master::ERROR_STATE);
    this->state = LIN_Master::STATE_DONE;
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
    this->state = LIN_Master::STATE_BODY;

  } // BREAK echo received
  
  // no byte(s) received
  else
  {
    // check for timeout
    if (micros() - this->timeStart > this->timeMax)
    {
      this->error = (LIN_Master::error_t) ((int) this->error | (int) LIN_Master::ERROR_TIMEOUT);
      this->state = LIN_Master::STATE_DONE;
    }

  } // no byte(s) received
  
  // return state
  return this->state;

} // LIN_Master_HardwareSerial_ESP8266::_sendFrame()



/**
  \brief      Constructor for LIN node class using ESP8266 HardwareSerial
  \details    Constructor for LIN node class for using ESP8266 HardwareSerial. Inherit all methods from LIN_Master_HardwareSerial, only different constructor
  \param[in]  SwapPins    use alternate Serial2 Rx/Tx pins 
  \param[in]  NameLIN     LIN node name 
*/
LIN_Master_HardwareSerial_ESP8266::LIN_Master_HardwareSerial_ESP8266(bool SwapPins, const char NameLIN[]) : LIN_Master_HardwareSerial::LIN_Master_HardwareSerial(Serial, NameLIN)
{
  // store parameters in class variables
  this->swapPins   = SwapPins;                                // use alternate pins Rx/Tx for Serial0 
  
  // must not open connection here, else system stalls

} // LIN_Master_HardwareSerial_ESP8266::LIN_Master_HardwareSerial_ESP8266()



/**
  \brief      Open serial interface
  \details    Open serial interface with specified baudrate. Here dummy!
  \param[in]  Baudrate    communication speed [Baud]
*/
void LIN_Master_HardwareSerial_ESP8266::begin(uint16_t Baudrate)
{
  // call base class method
  LIN_Master::begin(Baudrate);  

  // open serial interface
  this->pSerial->begin(baudrate, SERIAL_8N1);
  while(!(*(this->pSerial)));

  // route Serial0 to alternate pins
  if (this->swapPins == true)
    this->pSerial->swap();

} // LIN_Master_HardwareSerial_ESP8266::begin()

#endif // ARDUINO_ARCH_ESP8266

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/