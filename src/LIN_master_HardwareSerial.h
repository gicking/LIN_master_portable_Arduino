/**
  \file     LIN_master_HardwareSerial.h
  \brief    LIN master emulation library using a HardwareSerial interface
  \details  This library provides a master node emulation for a LIN bus via a HardwareSerial interface, optionally via RS485.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

/*-----------------------------------------------------------------------------
  MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _LIN_MASTER_HW_SERIAL_H_
#define _LIN_MASTER_HW_SERIAL_H_


/*-----------------------------------------------------------------------------
  INCLUDE FILES
-----------------------------------------------------------------------------*/

// include required libraries
#include <LIN_master_Base.h>


/*-----------------------------------------------------------------------------
  GLOBAL CLASS
-----------------------------------------------------------------------------*/
/**
  \brief  LIN master node class via HardwareSerial

  \details LIN master node class via HardwareSerial.
*/
class LIN_Master_HardwareSerial : public LIN_Master_Base
{
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
    LIN_Master_HardwareSerial(HardwareSerial &Interface, const char NameLIN[] = "Master", const int8_t PinTxEN = INT8_MIN);
     
    /// @brief Open serial interface
    void begin(uint16_t Baudrate = 19200);
    
    /// @brief Close serial interface
    void end(void);

}; // class LIN_Master_HardwareSerial


/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _LIN_MASTER_HW_SERIAL_H_

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
