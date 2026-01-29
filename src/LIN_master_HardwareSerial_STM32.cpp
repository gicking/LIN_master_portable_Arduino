/**
  \file     LIN_master_HardwareSerial_STM32.cpp
  \brief    LIN master emulation library using a HardwareSerial interface of STM32
  \details  This library provides a master node emulation for a LIN bus via a HardwareSerial interface of STM32, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \note     Have to use dedicated class due to long latency on baudrate change, see https://github.com/stm32duino/Arduino_Core_STM32/issues/2907
  \author   Georg Icking-Konert
*/

// assert STM32 platform
#if defined(ARDUINO_ARCH_STM32)

// include files
#include <LIN_master_HardwareSerial_STM32.h>


/**
  \brief      Send LIN break
  \details    Send LIN break (=16bit low)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial_STM32::_sendBreak(void)
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
  this->pSerial->flush();
  while (this->pSerial->available())
    this->pSerial->read();

  // if LIN mode is available clear BREAK detection flag for debugging purposes
  #if defined(USART_CR2_LINEN)
    this->huart->Instance->ICR = USART_ICR_LBDCF;
  #endif

  // optionally enable transmitter
  this->_enableTransmitter();

  // if LIN mode is available generate LIN BREAK (>=13 bit low)
  #if defined(USART_CR2_LINEN)
    this->huart->Instance->RQR |= USART_RQR_SBKRQ;

  // w/o LIN mode send 0x00 at 1/2 baudrate
  // Note: don't use Serial.begin() or TE=1 due to HW latency, see https://github.com/stm32duino/Arduino_Core_STM32/issues/2907#issuecomment-3816058235
  #else
    //this->huart->Instance->CR1 &= ~USART_CR1_UE;
    this->huart->Instance->BRR = this->brr * 2;
    //this->huart->Instance->CR1 |= USART_CR1_UE;
    this->pSerial->write((int) 0x00);
  #endif

  // progress state
  this->state = LIN_Master_Base::STATE_BREAK;

  // print debug message
  DEBUG_PRINT(3, " ");
  
  // return state
  return this->state;

} // LIN_Master_HardwareSerial_STM32::_sendBreak()



/**
  \brief      Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \details    Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial_STM32::_sendFrame(void)
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

  // byte(s) received (likely BREAK echo)
  if (this->pSerial->available())
  {
    // store echo in Rx
    this->bufRx[0] = this->pSerial->read();

    // if LIN mode is available do nothing. BREAK generated using dedicated LIN mode above
    #if defined(USART_CR2_LINEN)

    // w/o LIN mode revert baudrate
    // Note: don't use Serial.begin() or TE=1 due to HW latency, see https://github.com/stm32duino/Arduino_Core_STM32/issues/2907#issuecomment-3816058235
    #else
      this->huart->Instance->BRR = this->brr;
    #endif

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
      // print debug message
      DEBUG_PRINT(1, "Rx timeout");

      // set error state and return immediately
      this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_TIMEOUT);
      this->state = LIN_Master_Base::STATE_DONE;
      this->_disableTransmitter();
      return this->state;
    }

  } // no byte(s) received
  
  // print debug message
  DEBUG_PRINT(2, " ");
  
  // return state
  return this->state;

} // LIN_Master_HardwareSerial_STM32::_sendFrame()



/**
  \brief      Receive and check LIN frame
  \details    Receive and check LIN frame (request frame: check echo; response frame: check header echo & checksum)
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_HardwareSerial_STM32::_receiveFrame(void)
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

  // optionally disable RS485 transmitter for slave response frames. Len==2 because BREAK is handled already handled in _sendFrame()
  if ((this->type == LIN_Master_Base::SLAVE_RESPONSE) && (this->pSerial->available() == 2))
    this->_disableTransmitter();

  // frame body received (-1 because BREAK is handled already handled in _sendFrame())
  if (this->pSerial->available() >= this->lenRx-1)
  {
    // store bytes in Rx
    this->pSerial->readBytes(this->bufRx+1, this->lenRx-1);

    // check frame for errors
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) this->_checkFrame());

    // optionally disable RS485 transmitter after frame is completed
    this->_disableTransmitter();

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

} // LIN_Master_HardwareSerial_STM32::_receiveFrame()



/**
  \brief      Constructor for LIN node class using STM32 HardwareSerial
  \details    Constructor for LIN node class for using STM32 HardwareSerial. Store pointer to used serial interface w/ pins.
  \param[in]  Interface   serial interface for LIN
  \param[in]  PinRx       GPIO used for reception. See datasheet for optional pin remapping.
  \param[in]  PinTx       GPIO used for transmission. See datasheet for optional pin remapping.
  \param[in]  NameLIN     LIN node name (default = "Master")
  \param[in]  PinTxEN     optional Tx enable pin (high active) e.g. for LIN via RS485 (default = -127/none)
*/
LIN_Master_HardwareSerial_STM32::LIN_Master_HardwareSerial_STM32(HardwareSerial &Interface, uint32_t PinRx, 
  uint32_t PinTx, const char NameLIN[], const int8_t PinTxEN) : LIN_Master_Base::LIN_Master_Base(NameLIN, PinTxEN)
{
  // Debug serial initialized in begin() -> no debug output here

  // store pointer to used HW serial
  this->pSerial    = &Interface;                              // used serial interface
  this->huart      = Interface.getHandle();                   // pointer to underlying HAL UART handle
  this->pinRx      = PinRx;                                   // receive pin
  this->pinTx      = PinTx;                                   // transmit pin

} // LIN_Master_HardwareSerial_STM32::LIN_Master_HardwareSerial_STM32()



/**
  \brief      Open serial interface
  \details    Open serial interface with specified baudrate
  \param[in]  Baudrate    communication speed [Baud] (default = 19200)
*/
void LIN_Master_HardwareSerial_STM32::begin(uint16_t Baudrate)
{
  // call base class method
  LIN_Master_Base::begin(Baudrate);

  // just to be sure
  this->pSerial->end();

  // remap Rx/Tx pins (before begin()!)
  this->pSerial->setTx(this->pinTx);
  this->pSerial->setRx(this->pinRx);

  // open serial interface with optional timeout
  this->pSerial->begin(this->baudrate);
  #if defined(LIN_MASTER_LIN_PORT_TIMEOUT) && (LIN_MASTER_LIN_PORT_TIMEOUT > 0)
    uint32_t startMillis = millis();
    while ((!(*(this->pSerial))) && (millis() - startMillis < LIN_MASTER_LIN_PORT_TIMEOUT));
  #else
    while(!(*(this->pSerial)));
  #endif

  // if LIN mode is available enable LIN mode -> BREAK 13b & enable BREAK detection
  #if defined(USART_CR2_LINEN)
    this->huart->Instance->CR1 &= ~USART_CR1_UE;
    this->huart->Instance->CR2 |= USART_CR2_LINEN;
    this->huart->Instance->CR1 |= USART_CR1_UE;
  
  // w/o LIN mode store BRR value for BRK generation
  #else
    this->brr = this->huart->Instance->BRR;
  #endif

  // print debug message
  DEBUG_PRINT(2, "ok");

} // LIN_Master_HardwareSerial_STM32::begin()



/**
  \brief      Close serial interface
  \details    Close serial interface
*/
void LIN_Master_HardwareSerial_STM32::end()
{
  // call base class method
  LIN_Master_Base::end();
    
  // close serial interface
  this->pSerial->end();

  // disable LIN mode
  this->huart->Instance->CR1 &= ~USART_CR1_UE;
  this->huart->Instance->CR2 &= ~USART_CR2_LINEN;
  this->huart->Instance->CR1 |= USART_CR1_UE;

  // print debug message
  DEBUG_PRINT(2, " ");

} // LIN_Master_HardwareSerial_STM32::end()

#endif // ARDUINO_ARCH_STM32

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
