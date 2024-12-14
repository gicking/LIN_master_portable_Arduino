/**
  \file     LIN_master_SoftwareSerial.h
  \brief    LIN master emulation library for SoftwareSerial
  \details  This library provides a master node emulation for a LIN bus via SoftwareSerial, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

// assert platform which supports SoftwareSerial. Note: ARDUINO_ARCH_ESP32 requires library ESPSoftwareSerial
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) 

/*-----------------------------------------------------------------------------
  MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _LIN_MASTER_SW_SERIAL_H_
#define _LIN_MASTER_SW_SERIAL_H_


/*-----------------------------------------------------------------------------
  INCLUDE FILES
-----------------------------------------------------------------------------*/

// include required libraries
#include <LIN_master_Base.h>
#include <SoftwareSerial.h>


/*-----------------------------------------------------------------------------
  GLOBAL CLASS
-----------------------------------------------------------------------------*/
/**
  \brief  LIN master node class via SoftwareSerial

  \details LIN master node class via SoftwareSerial.
*/
class LIN_Master_SoftwareSerial : public LIN_Master_Base
{
  // PRIVATE VARIABLES
  private:

    uint8_t               pinRx;              //!< pin used for receive
    uint8_t               pinTx;              //!< pin used for transmit
    bool                  inverseLogic;       //!< use inverse logic
    uint32_t              durationBreak;      //!< duration [us] of sync break


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
    LIN_Master_SoftwareSerial(uint8_t PinRx, uint8_t PinTx, bool InverseLogic = false, const char NameLIN[] = "Master", const int8_t PinTxEN = INT8_MIN);
     
    /// @brief Open serial interface
    void begin(uint16_t Baudrate = 19200);
    
    /// @brief Close serial interface
    void end(void);

}; // class LIN_Master_SoftwareSerial


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _LIN_MASTER_SW_SERIAL_H_

#endif // ARDUINO_ARCH_AVR || ARDUINO_ARCH_ESP8266 || ARDUINO_ARCH_ESP32

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
