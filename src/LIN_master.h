/**
  \file     LIN_master.h
  \brief    Base class for LIN master emulation
  \details  This library provides the base class for a master node emulation of a LIN bus.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

/*-----------------------------------------------------------------------------
  MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _LIN_MASTER_H_
#define _LIN_MASTER_H_


/*-----------------------------------------------------------------------------
  GLOBAL DEFINES
-----------------------------------------------------------------------------*/

#define BUFLEN_NAME        30           //!< max. length of node name

//#define LIN_DEBUG_SERIAL   Serial       //!< Serial interface used for debug output
//#define LIN_DEBUG_LEVEL    2            //!< Debug level (0=no output, 1=error msg, 2=sent/received bytes)


/*-----------------------------------------------------------------------------
  INCLUDE FILES
-----------------------------------------------------------------------------*/

#include <Arduino.h>


/*-----------------------------------------------------------------------------
  GLOBAL CLASS
-----------------------------------------------------------------------------*/

/**
  \brief  LIN master node base class

  \details LIN master node base class. From this class the actual LIN classes for a Serialx are derived.
*/
class LIN_Master
{
  // PUBLIC TYPEDEFS
  public:

    /// LIN protocol version 
    typedef enum
    {
        LIN_V1 = 1,                               //!< LIN protocol version 1.x
        LIN_V2 = 2                                //!< LIN protocol version 2.x
    } version_t;


    /// LIN frame type
    typedef enum
    {
        MASTER_REQUEST = 1,                       //!< LIN master request frame
        SLAVE_RESPONSE = 2                        //!< LIN slave response frame
    } frame_t;


    /// state of LIN master state machine
    typedef enum
    {
        STATE_OFF     = 0,                        //!< LIN interface closed
        STATE_IDLE    = 1,                        //!< no LIN transmission ongoing
        STATE_BREAK   = 2,                        //!< sync break is being transmitted
        STATE_BODY    = 3,                        //!< rest of frame is being sent/received
        STATE_DONE    = 4,                        //!< frame completed
    } state_t;


    /// LIN error codes. Use bitmasks, as error is latched
    typedef enum
    {
        NO_ERROR      = 0x00,                     //!< no error
        ERROR_STATE   = 0x01,                     //!< error in LIN state machine
        ERROR_ECHO    = 0x02,                     //!< error reading LIN echo
        ERROR_TIMEOUT = 0x04,                     //!< frame timeout error
        ERROR_CHK     = 0x08,                     //!< LIN checksum error
        ERROR_MISC    = 0x80                      //!< misc error, should not occur
    } error_t;


  // PROTECTED VARIABLES
  protected:

    // node properties
    uint16_t              baudrate;               //!< communication baudrate [Baud]
    LIN_Master::state_t   state;                  //!< status of LIN state machine
    LIN_Master::error_t   error;                  //!< error state. Is latched until cleared
    uint32_t              timePerByte;            //!< time [us] per byte at specified baudrate
    uint32_t              timeStart;              //!< starting time [us] for frame timeout
    uint32_t              timeMax;                //!< max. frame duration [us]

    // frame properties
    LIN_Master::version_t version;                //!< LIN protocol version
    LIN_Master::frame_t   type;                   //!< LIN frame type
    uint8_t               id;                     //!< LIN frame identifier (protected or unprotected)
    uint8_t               lenTx;                  //!< send buffer length (max. 12)
    uint8_t               bufTx[12];              //!< send buffer incl. BREAK, SYNC, DATA and CHK (max. 12B)
    uint8_t               lenRx;                  //!< receive buffer length (max. 12)
    uint8_t               bufRx[12];              //!< receive buffer incl. BREAK, SYNC, DATA and CHK (max. 12B)


  // PUBLIC VARIABLES
  public:

    char                  nameLIN[BUFLEN_NAME];   //!< LIN node name, e.g. for debug


  // PROTECTED METHODS
  protected:
  
    /// @brief Calculate protected frame ID
    uint8_t _calculatePID(void);

    /// @brief Calculate LIN frame checksum
    uint8_t _calculateChecksum(uint8_t NumData, uint8_t Data[]);

    /// @brief Check received LIN frame
    LIN_Master::error_t _checkFrame(void);

    /// @brief Send LIN break
    virtual LIN_Master::state_t _sendBreak(void);

    /// @brief Send LIN frame body
    virtual LIN_Master::state_t _sendFrame(void);

    /// @brief Receive LIN frame
    virtual LIN_Master::state_t _receiveFrame(void);


  // PUBLIC METHODS
  public:
  
    /// @brief LIN master node constructor
    LIN_Master(const char NameLIN[]);
    
    
    /// @brief Open serial interface
    virtual void begin(uint16_t Baudrate);
    
    /// @brief Close serial interface
    virtual void end(void);
    
    
    /// @brief Reset LIN state machine
    inline void resetStateMachine(void) { this->state = LIN_Master::STATE_IDLE; }
    
    /// @brief Getter for LIN state machine state
    inline LIN_Master::state_t getState(void) { return this->state; }

    
    /// @brief Clear error of LIN state machine
    inline void resetError(void) { this->error = LIN_Master::NO_ERROR; }
    
    /// @brief Getter for LIN state machine error
    inline LIN_Master::error_t getError(void) { return this->error; }
    
    
    /// @brief Getter for LIN frame
    inline void getFrame(LIN_Master::frame_t &Type, uint8_t &Id, uint8_t &NumData, uint8_t Data[])
    { 
      noInterrupts();               // for data consistency temporarily disable ISRs
      Type    = this->type;         // frame type 
      Id      = this->id;           // frame ID
      NumData = this->lenRx - 4;    // number of data bytes (excl. BREAK, SYNC, ID, CHK)
      memcpy(Data, this->bufRx+3, NumData);
      interrupts();                 // re-enable ISRs
    }

    
    /// @brief Start sending a LIN master request frame in background (if supported)
    LIN_Master::state_t sendMasterRequest(LIN_Master::version_t Version, uint8_t Id, uint8_t NumData, uint8_t Data[]);
    
    /// @brief Send a blocking LIN master request frame (no background operation)
    LIN_Master::error_t sendMasterRequestBlocking(LIN_Master::version_t Version, uint8_t Id, uint8_t NumData, uint8_t Data[]);

    /// @brief Start sending a LIN slave response frame in background (if supported)
    LIN_Master::state_t receiveSlaveResponse(LIN_Master::version_t Version, uint8_t Id, uint8_t NumData);
    
    /// @brief Send a blocking LIN slave response frame (no background operation)
    LIN_Master::error_t receiveSlaveResponseBlocking(LIN_Master::version_t Version, uint8_t Id, uint8_t NumData, uint8_t *Data);

    /// @brief Handle LIN background operation (call until STATE_DONE is returned)
    LIN_Master::state_t handler(void);

}; // class LIN_Master

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _LIN_MASTER_H_
