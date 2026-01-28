/**
  \file     LIN_master_HardwareSerial_STM32.h
  \brief    LIN master emulation library using a HardwareSerial interface of STM32
  \details  This library provides a master node emulation for a LIN bus via a HardwareSerial interface of STM32, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

// assert STM32 platform
#if defined(ARDUINO_ARCH_STM32)

/*-----------------------------------------------------------------------------
  MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _LIN_MASTER_HW_SERIAL_STM32_H_
#define _LIN_MASTER_HW_SERIAL_STM32_H_


/*-----------------------------------------------------------------------------
  INCLUDE FILES
-----------------------------------------------------------------------------*/

// include required libraries
#include <LIN_master_Base.h>


/*-----------------------------------------------------------------------------
  GLOBAL CLASS
-----------------------------------------------------------------------------*/
/**
  \brief  LIN master node class via STM32 HardwareSerial

  \details LIN master node class via STM32 HardwareSerial.
*/
class LIN_Master_HardwareSerial_STM32 : public LIN_Master_Base
{
  // PROTECTED VARIABLES
  protected:

    HardwareSerial        *pSerial;           //!< serial interface used for LIN
    UART_HandleTypeDef    *huart;             //!< pointer to underlying HAL UART handle
    uint32_t              pinRx;              //!< pin used for receive
    uint32_t              pinTx;              //!< pin used for transmit

  // PROTECTED METHODS
  protected:
  
    /// @brief Send LIN break
    LIN_Master_Base::state_t _sendBreak(void);

    /// @brief Send LIN bytes (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID)
    LIN_Master_Base::state_t _sendFrame(void);

    /// @brief Read and check LIN frame
    LIN_Master_Base::state_t _receiveFrame(void);


  // PUBLIC METHODS
  public:

    /// @brief Class constructor
    LIN_Master_HardwareSerial_STM32(HardwareSerial &Interface, uint32_t PinRx, uint32_t PinTx,
      const char NameLIN[] = "Master", const int8_t PinTxEN = INT8_MIN);
     
    /// @brief Open serial interface
    void begin(uint16_t Baudrate = 19200);
    
    /// @brief Close serial interface
    void end(void);

}; // class LIN_master_HardwareSerial_STM32


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _LIN_MASTER_HW_SERIAL_STM32_H_

#endif // ARDUINO_ARCH_STM32

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
