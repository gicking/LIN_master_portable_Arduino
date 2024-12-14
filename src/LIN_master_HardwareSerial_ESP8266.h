/**
  \file     LIN_master_HardwareSerial_ESP8266.h
  \brief    LIN master emulation library using hardware Serial0 interface of ESP8266
  \details  This library provides a master node emulation for a LIN bus via hardware Serial0 interface of ESP8266, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

// assert ESP8266 platform
#if defined(ARDUINO_ARCH_ESP8266)

/*-----------------------------------------------------------------------------
  MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _LIN_MASTER_HW_SERIAL_ESP8266_H_
#define _LIN_MASTER_HW_SERIAL_ESP8266_H_


/*-----------------------------------------------------------------------------
  INCLUDE FILES
-----------------------------------------------------------------------------*/

// include required libraries
#include <LIN_master_Base.h>


/*-----------------------------------------------------------------------------
  GLOBAL CLASS
-----------------------------------------------------------------------------*/
/**
  \brief  LIN master node class via ESP8266 HardwareSerial

  \details LIN master node class via ESP8266 HardwareSerial.
*/
class LIN_Master_HardwareSerial_ESP8266 : public LIN_Master_Base
{
  // PROTECTED VARIABLES
  protected:

    bool                  swapPins;           //!< use alternate pins for Serial0


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
    LIN_Master_HardwareSerial_ESP8266(bool SwapPins = false, const char NameLIN[] = "Master", const int8_t PinTxEN = INT8_MIN);
     
    /// @brief Open serial interface
    void begin(uint16_t Baudrate = 19200);
    
    /// @brief Close serial interface
    void end(void);

}; // class LIN_master_HardwareSerial_ESP8266


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _LIN_MASTER_HW_SERIAL_ESP8266_H_

#endif // ARDUINO_ARCH_ESP8266

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
