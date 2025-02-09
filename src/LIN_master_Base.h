/**
  \file     LIN_master_Base.h
  \brief    Base class for LIN master emulation (non-functional)
  \details  This library provides the base class for a master node emulation of a LIN bus.
            The base class is non-functional, as it lacks the actual communication interface.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

/*-----------------------------------------------------------------------------
  MODULE DEFINITION FOR MULTIPLE INCLUSION
-----------------------------------------------------------------------------*/
#ifndef _LIN_MASTER_BASE_H_
#define _LIN_MASTER_BASE_H_


/*-----------------------------------------------------------------------------
  GLOBAL DEFINES
-----------------------------------------------------------------------------*/

// misc parameters
#define LIN_MASTER_BUFLEN_NAME  30            //!< max. length of node name

// optional debug output @ 115.2kBaud. When using together with NeoHWSerial on AVR must use NeoSerialx to avoid linker conflict
#if !defined(LIN_MASTER_DEBUG_SERIAL)
  //#define LIN_MASTER_DEBUG_SERIAL Serial        //!< serial interface used for debug output. Comment out for none
  //#define LIN_MASTER_DEBUG_SERIAL NeoSerial     //!< serial interface used for debug output (required for AVR). Comment out for none
#endif
#if !defined(LIN_MASTER_DEBUG_LEVEL)
  #define LIN_MASTER_DEBUG_LEVEL  2             //!< debug verbosity 0..3 (1=errors only, 3=chatty)
#endif

/*-----------------------------------------------------------------------------
  INCLUDE FILES
-----------------------------------------------------------------------------*/

// generic Arduino functions
#include <Arduino.h>

// required for debug on AVR via NeoSerial
#if defined (LIN_SLAVE_DEBUG_SERIAL) && (LIN_SLAVE_DEBUG_SERIAL == NeoSerial)
  #if defined(ARDUINO_ARCH_AVR)
    #include <NeoHWSerial.h>
  #else
    #error NeoSerial only available for AVR
  #endif
#endif


/*-----------------------------------------------------------------------------
  GLOBAL CLASS
-----------------------------------------------------------------------------*/

/**
  \brief  LIN master node base class

  \details LIN master node base class. From this class the actual LIN classes for a Serialx are derived.
*/
class LIN_Master_Base
{
  // PUBLIC TYPEDEFS
  public:

    /// LIN protocol version 
    typedef enum : uint8_t
    {
      LIN_V1                = 1,                //!< LIN protocol version 1.x
      LIN_V2                = 2                 //!< LIN protocol version 2.x
    } version_t;


    /// LIN frame type
    typedef enum : uint8_t
    {
      MASTER_REQUEST        = 0x01,             //!< LIN master request frame
      SLAVE_RESPONSE        = 0x02              //!< LIN slave response frame
    } frame_t;


    /// state of LIN master state machine. Use bitmasks for fast checking multiple states
    typedef enum : uint8_t
    {
      STATE_OFF             = 0x01,             //!< LIN interface closed
      STATE_IDLE            = 0x02,             //!< no LIN transmission ongoing
      STATE_BREAK           = 0x04,             //!< sync break is being transmitted
      STATE_BODY            = 0x08,             //!< rest of frame is being sent/received
      STATE_DONE            = 0x10              //!< frame completed
    } state_t;


    /// LIN error codes. Use bitmasks, as error is latched. Use same as LIN_slave_portable
    typedef enum : uint8_t
    {
      NO_ERROR              = 0x00,             //!< no error
      ERROR_STATE           = 0x01,             //!< error in LIN state machine
      ERROR_ECHO            = 0x02,             //!< error reading response echo
      ERROR_TIMEOUT         = 0x04,             //!< frame timeout error
      ERROR_CHK             = 0x08,             //!< LIN checksum error
      ERROR_MISC            = 0x80              //!< misc error, should not occur
    } error_t;


  // PROTECTED VARIABLES
  protected:

    // node properties
    int8_t                  pinTxEN;            //!< optional Tx direction pin, e.g. for LIN via RS485 
    uint16_t                baudrate;           //!< communication baudrate [Baud]
    LIN_Master_Base::state_t  state;            //!< status of LIN state machine
    LIN_Master_Base::error_t  error;            //!< error state. Is latched until cleared
    uint32_t                timePerByte;        //!< time [us] per byte at specified baudrate
    uint32_t                timeoutFrame;       //!< max. frame duration [us]

    // frame properties
    LIN_Master_Base::version_t  version;        //!< LIN protocol version
    LIN_Master_Base::frame_t  type;             //!< LIN frame type
    uint8_t                 id;                 //!< LIN frame identifier (protected or unprotected)
    uint8_t                 lenTx;              //!< send buffer length (max. 12)
    uint8_t                 bufTx[12];          //!< send buffer incl. BREAK, SYNC, DATA and CHK (max. 12B)
    uint8_t                 lenRx;              //!< receive buffer length (max. 12)
    uint8_t                 bufRx[12];          //!< receive buffer incl. BREAK, SYNC, DATA and CHK (max. 12B)
    uint32_t                timeStart;          //!< starting time [us] for frame timeout


  // PUBLIC VARIABLES
  public:

    char                    nameLIN[LIN_MASTER_BUFLEN_NAME];  //!< LIN node name, e.g. for debug


  // PROTECTED METHODS
  protected:
  
    /// @brief Calculate protected frame ID
    uint8_t _calculatePID(void);

    /// @brief Calculate LIN frame checksum
    uint8_t _calculateChecksum(uint8_t NumData, uint8_t Data[]);

    /// @brief Check received LIN frame
    LIN_Master_Base::error_t _checkFrame(void);

    /// @brief Send LIN break
    virtual LIN_Master_Base::state_t _sendBreak(void);

    /// @brief Send LIN frame body
    virtual LIN_Master_Base::state_t _sendFrame(void);

    /// @brief Receive LIN frame
    virtual LIN_Master_Base::state_t _receiveFrame(void);
    

    /// @brief Enable RS485 transmitter (DE=high)
    inline void _enableTransmitter(void)
    {   
      // print debug message (debug level 3)
      #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 3)
        LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::_enableTransmitter()");
      #endif

      // enable tranmitter
      if (this->pinTxEN >= 0)
        digitalWrite(this->pinTxEN, HIGH);
    
    } // _enableTransmitter()
    
    /// @brief Disable RS485 transmitter (DE=low)
    inline void _disableTransmitter(void)
    { 
      // print debug message (debug level 3)
      #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 3)
        LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::_disableTransmitter()");
      #endif

      // disable tranmitter
      if (this->pinTxEN >= 0)
        digitalWrite(this->pinTxEN, LOW);
            
    } // _disableTransmitter()


  // PUBLIC METHODS
  public:
  
    /// @brief LIN master node constructor
    LIN_Master_Base(const char NameLIN[] = "Master", const int8_t PinTxEN = INT8_MIN);

    /// @brief LIN master node destructor, here dummy. Any class with virtual functions should have virtual destructor
    virtual ~LIN_Master_Base(void) { }


    /// @brief Open serial interface
    virtual void begin(uint16_t Baudrate = 19200);
    
    /// @brief Close serial interface
    virtual void end(void);
    
    
    /// @brief Reset LIN state machine
    inline void resetStateMachine(void)
    {
      // print debug message (debug level 3)
      #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 3)
        LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::resetStateMachine()");
      #endif

      // reset state
      this->state = LIN_Master_Base::STATE_IDLE;

    } // resetStateMachine()
    
    /// @brief Getter for LIN state machine state
    inline LIN_Master_Base::state_t getState(void)
    {
      // print debug message (debug level 3)
      #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 3)
        LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::getState()");
      #endif

      // return state
      return this->state;
      
    } // getState()

    
    /// @brief Clear error of LIN state machine
    inline void resetError(void) 
    {
      // print debug message (debug level 3)
      #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 3)
        LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::resetError()");
      #endif

      // reset error
      this->error = LIN_Master_Base::NO_ERROR;

    } // resetError()
    
    /// @brief Getter for LIN state machine error
    inline LIN_Master_Base::error_t getError(void)
    {
      // print debug message (debug level 3)
      #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 3)
        LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::getError()");
      #endif

      // return error
      return this->error;

    } // getError()
    

    /// @brief Getter for LIN frame
    inline void getFrame(LIN_Master_Base::frame_t &Type, uint8_t &Id, uint8_t &NumData, uint8_t Data[])
    { 
      // print debug message (debug level 3)
      #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 3)
        LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::getFrame()");
      #endif

      noInterrupts();                         // for data consistency temporarily disable ISRs
      Type    = this->type;                   // frame type 
      Id      = this->id;                     // frame ID
      NumData = this->lenRx - 4;              // number of data bytes (excl. BREAK, SYNC, ID, CHK)
      memcpy(Data, this->bufRx+3, NumData);   // copy data bytes
      interrupts();                           // re-enable ISRs

    } // getFrame()

    
    /// @brief Start sending a LIN master request frame in background (if supported)
    LIN_Master_Base::state_t sendMasterRequest(LIN_Master_Base::version_t Version = LIN_Master_Base::LIN_V2, 
      uint8_t Id = 0x00, uint8_t NumData = 0, uint8_t Data[] = NULL);
    
    /// @brief Send a blocking LIN master request frame (no background operation)
    LIN_Master_Base::error_t sendMasterRequestBlocking(LIN_Master_Base::version_t Version = LIN_Master_Base::LIN_V2, 
      uint8_t Id = 0x00, uint8_t NumData = 0, uint8_t Data[] = NULL);

    /// @brief Start sending a LIN slave response frame in background (if supported)
    LIN_Master_Base::state_t receiveSlaveResponse(LIN_Master_Base::version_t Version = LIN_Master_Base::LIN_V2, 
      uint8_t Id = 0x00, uint8_t NumData = 0);
    
    /// @brief Send a blocking LIN slave response frame (no background operation)
    LIN_Master_Base::error_t receiveSlaveResponseBlocking(LIN_Master_Base::version_t Version = LIN_Master_Base::LIN_V2,
      uint8_t Id = 0x00, uint8_t NumData = 0, uint8_t *Data = NULL);

    /// @brief Handle LIN background operation (call until STATE_DONE is returned)
    LIN_Master_Base::state_t handler(void);

}; // class LIN_Master_Base

/*-----------------------------------------------------------------------------
    END OF MODULE DEFINITION FOR MULTIPLE INLUSION
-----------------------------------------------------------------------------*/
#endif // _LIN_MASTER_BASE_H_

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
