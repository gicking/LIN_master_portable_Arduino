/**
  \file     LIN_master_Base.cpp
  \brief    Base class for LIN master emulation
  \details  This library provides the base class for a master node emulation of a LIN bus.
            For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \author   Georg Icking-Konert
*/

// include files
#include <LIN_master_Base.h>


/**************************
 * PROTECTED METHODS
**************************/

/**
  \brief      Calculate protected frame ID
  \details    Calculate protected frame ID as described in LIN2.0 spec "2.3.1.3 Protected identifier field"
  \return     Protected frame ID
*/
uint8_t LIN_Master_Base::_calculatePID(void)
{
  uint8_t  pid;       // protected frame ID
  uint8_t  tmp;       // temporary variable for calculating parity bits

  // copy (unprotected) ID
  pid = this->id;

  // protect ID  with parity bits
  pid  = (uint8_t) (pid & 0x3F);                                            // clear upper bits 6 & 7
  tmp  = (uint8_t) ((pid ^ (pid>>1) ^ (pid>>2) ^ (pid>>4)) & 0x01);         // pid[6] = PI0 = ID0^ID1^ID2^ID4
  pid |= (uint8_t) (tmp << 6);
  tmp  = (uint8_t) (~((pid>>1) ^ (pid>>3) ^ (pid>>4) ^ (pid>>5)) & 0x01);   // pid[7] = PI1 = ~(ID1^ID3^ID4^ID5)
  pid |= (uint8_t) (tmp << 7);

  // return protected ID
  return pid;

} // LIN_Master_Base::_calculatePID()



/**
  \brief      Calculate LIN frame checksum
  \details    Calculate LIN frame checksum as described in LIN1.x / LIN2.x specs
  \param[in]  NumData   number of data bytes in frame
  \param[in]  Data      frame data bytes
  \return     calculated checksum, depending on protocol version
*/
uint8_t LIN_Master_Base::_calculateChecksum(uint8_t NumData, uint8_t Data[])
{
  uint8_t  pid;       // protected frame ID
  uint16_t chk=0x00;

  // calculate protected frame ID
  pid = this->_calculatePID();

  // LIN2.x uses extended checksum which includes protected ID, i.e. including parity bits
  // LIN1.x uses classical checksum only over data bytes
  // Diagnostic frames with ID 0x3C and 0x3D/0x7D always use classical checksum (see LIN spec "2.3.1.5 Checkum")
  if (!((this->version == LIN_V1) || (pid == 0x3C) || (pid == 0x7D)))    // if version 2  & no diagnostic frames (0x3C=60 (PID=0x3C) or 0x3D=61 (PID=0x7D))
    chk = (uint16_t) pid;

  // loop over data bytes
  for (uint8_t i = 0; i < NumData; i++)
  {
    chk += (uint16_t) (Data[i]);
    if (chk>255)
      chk -= 255;
  }
  chk = (uint8_t)(0xFF - ((uint8_t) chk));   // bitwise invert

  // return frame checksum
  return (uint8_t) chk;

} // LIN_Master_Base::_calculateChecksum()



/**
  \brief      Check received LIN frame
  \details    Check received LIN frame for echo and frame checksum
  \return     error check result
*/
LIN_Master_Base::error_t LIN_Master_Base::_checkFrame(void)
{
  // print debug message
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::_receiveFrame()");

    // print data to check
    LIN_MASTER_DEBUG_SERIAL.print('\t');
    for (uint8_t i=0; i<this->lenTx; i++)
    {
      LIN_MASTER_DEBUG_SERIAL.print((int) i);
      LIN_MASTER_DEBUG_SERIAL.print('\t');
      LIN_MASTER_DEBUG_SERIAL.print(this->bufTx[i]);
      LIN_MASTER_DEBUG_SERIAL.print('\t');
      LIN_MASTER_DEBUG_SERIAL.print(this->bufRx[i]);
      LIN_MASTER_DEBUG_SERIAL.println();
    }
    LIN_MASTER_DEBUG_SERIAL.println();

  #endif


  // check echo of sent bytes (frame or header). Exit on mismatch
  for (uint8_t i=0; i<this->lenTx; i++)
    if (this->bufTx[i] != this->bufRx[i])
      return LIN_Master_Base::ERROR_ECHO;

  // check frame checksum
  if (this->bufRx[lenRx-1] != _calculateChecksum(this->lenRx-4, this->bufRx+3))
    return LIN_Master_Base::ERROR_CHK;

  // return result of check
  return LIN_Master_Base::NO_ERROR;

} // LIN_Master_Base::_checkFrame()



/**
  \brief      Send LIN break
  \details    Send LIN break (=16bit low). Here dummy!
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_Base::_sendBreak(void)
{
  // print debug message
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::_sendBreak()");
  #endif
    
  // if bus is not idle, return immediately
  if (this->state != LIN_Master_Base::STATE_IDLE)
  {
    this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_STATE);
    this->state = LIN_Master_Base::STATE_DONE;
    return this->state;
  }

  // progress state
  this->state = LIN_Master_Base::STATE_BREAK;

  // return state machine state
  return this->state;

} // LIN_Master_Base::_sendBreak()



/**
  \brief      Send LIN frame body
  \details    Send LIN frame body (request frame: SYNC+ID+DATA[]+CHK; response frame: SYNC+ID). Here dummy!
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_Base::_sendFrame(void)
{
  // print debug message
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::_sendFrame()");
  #endif

  // progress state
  this->state = LIN_Master_Base::STATE_BODY;

  // return state
  return this->state;

} // LIN_Master_Base::_sendFrame()



/**
  \brief      Receive and check LIN frame
  \details    Receive and check LIN frame (request frame: check echo; response frame: check header echo & checksum). Here dummy!
  \return     current state of LIN state machine
*/
LIN_Master_Base::state_t LIN_Master_Base::_receiveFrame(void)
{
  // print debug message
  #if defined(LIN_MASTER_DEBUG_SERIAL) && (LIN_MASTER_DEBUG_LEVEL >= 2)
    LIN_MASTER_DEBUG_SERIAL.println("LIN_Master_Base::_receiveFrame()");
  #endif

  // dummy: just progress state
  disableTransmitter();
  this->state = LIN_Master_Base::STATE_DONE;

  // return state
  return this->state;

} // LIN_Master_Base::_receiveFrame()



/**************************
 * PUBLIC METHODS
**************************/

/**
  \brief      LIN master node constructor
  \details    LIN master node constructor. Initialize class variables to default values.
              For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network.
              Optional direction switching via TxEn pin, e.g. for LIN via RS485  
  \param[in]  NameLIN     LIN node name (default = "Master")
  \param[in]  PinTxEN     optional Tx enable pin (high active) e.g. for LIN via RS485 (default = -127/none)
*/
LIN_Master_Base::LIN_Master_Base(const char NameLIN[], const int8_t PinTxEN)
{
  // store parameters in class variables
  memcpy(this->nameLIN, NameLIN, LIN_MASTER_BUFLEN_NAME);     // node name e.g. for debug
  this->pinTxEN = PinTxEN;

  // initialize master node properties
  this->error = LIN_Master_Base::NO_ERROR;                         // last LIN error. Is latched
  this->state = LIN_Master_Base::STATE_OFF;                        // status of LIN state machine

  // initialize TxEN pin low (=transmitter off)
  if (this->pinTxEN >= 0)
  {
    digitalWrite(this->pinTxEN, LOW);
    pinMode(this->pinTxEN, OUTPUT);
  }

} // LIN_Master_Base::LIN_Master_Base()



/**
  \brief      Open serial interface
  \details    Open serial interface with specified baudrate. Here dummy!
  \param[in]  Baudrate    communication speed [Baud] (default = 19200)
*/
void LIN_Master_Base::begin(uint16_t Baudrate)
{
  // store parameters in class variables
  this->baudrate   = Baudrate;                                // communication baudrate [Baud]

  // initialize master node properties
  this->error = LIN_Master_Base::NO_ERROR;                         // last LIN error. Is latched
  this->state = LIN_Master_Base::STATE_IDLE;                       // status of LIN state machine
  this->timePerByte = 10000000L / (uint32_t) this->baudrate;  // time [us] per byte (for performance)

} // LIN_Master_Base::begin()



/**
  \brief      Close serial interface
  \details    Close serial interface. Here dummy!
*/
void LIN_Master_Base::end()
{
  // initialize master node properties
  this->state = LIN_Master_Base::STATE_OFF;                        // status of LIN state machine

} // LIN_Master_Base::end()



/**
  \brief      Start sending a LIN master request frame in background (if supported)
  \details    Start sending a LIN master request frame in background (if supported). Background handling is handling by handler().
              For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \param[in]  Version   LIN protocol version
  \param[in]  Id        frame idendifier (protected or unprotected)
  \param[in]  NumData   number of data bytes (0..8)
  \param[in]  Data      data bytes
  \return     LIN state machine state
*/
LIN_Master_Base::state_t LIN_Master_Base::sendMasterRequest(LIN_Master_Base::version_t Version, uint8_t Id, uint8_t NumData, uint8_t Data[])
{
  // construct Tx frame
  this->type     = LIN_Master_Base::MASTER_REQUEST;
  this->version  = Version;
  this->id       = Id;
  this->lenTx    = NumData + 4;                                     // Frame length
  this->bufTx[0] = 0x00;                                            // BREAK
  this->bufTx[1] = 0x55;                                            // SYNC
  this->bufTx[2] = this->_calculatePID();                           // PID
  memcpy(this->bufTx+3, Data, NumData);                             // DATA[]
  this->bufTx[this->lenTx-1] = _calculateChecksum(NumData, Data);   // CHK
  this->lenRx    = this->lenTx;                                     // just receive LIN echo

  // init receive buffer
  memset(this->bufRx, 0, 12);

  // set break timeout (= 150% nominal) and start timeout
  this->timeStart = micros();
  this->timeMax   = (((this->lenRx + 1) * this->timePerByte) * 3 ) >> 1;

  // start LIN frame by sending a Sync Break
  this->_sendBreak();

  // return state machine state
  return this->state;

} // LIN_Master_Base::sendMasterRequest()



/**
  \brief      Send a blocking LIN master request frame (no background operation)
  \details    Send a blocking LIN master request frame (no background operation).
              For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \param[in]  Version   LIN protocol version
  \param[in]  Id        frame idendifier (protected or unprotected)
  \param[in]  NumData   number of data bytes (0..8)
  \param[in]  Data      data bytes
  \return     LIN error
*/
LIN_Master_Base::error_t LIN_Master_Base::sendMasterRequestBlocking(LIN_Master_Base::version_t Version, uint8_t Id, uint8_t NumData, uint8_t Data[])
{
  // start master request frame
  this->sendMasterRequest(Version, Id, NumData, Data);
  
  // wait until frame is completed
  do
    this->handler();
  while (this->state != LIN_Master_Base::STATE_DONE);

  // return LIN error
  return this->error;

} // LIN_Master_Base::sendMasterRequestBlocking()



/**
  \brief      Send/receive a blocking LIN slave response frame (no background operation)
  \details    Send/receive a blocking LIN slave response frame (no background operation).
              For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \param[in]  Version   LIN protocol version
  \param[in]  Id        frame idendifier (protected or unprotected)
  \param[in]  NumData   number of data bytes (0..8)
  \return     LIN state machine state
*/
LIN_Master_Base::state_t LIN_Master_Base::receiveSlaveResponse(LIN_Master_Base::version_t Version, uint8_t Id, uint8_t NumData)
{
  // construct Tx frame
  this->type     = LIN_Master_Base::SLAVE_RESPONSE;
  this->version  = Version;
  this->id       = Id;
  this->lenTx    = 3;                                               // Frame header length
  this->bufTx[0] = 0x00;                                            // BREAK
  this->bufTx[1] = 0x55;                                            // SYNC
  this->bufTx[2] = this->_calculatePID();                           // PID
  this->lenRx    = NumData + 4;                                     // receive LIN header echo + DATA[] + CHK

  // init receive buffer
  memset(this->bufRx, 0, 12);

  // set break timeout (= 150% nominal) and start timeout
  this->timeMax   = (((this->lenRx + 1) * this->timePerByte) * 3 ) >> 1;
  this->timeStart = micros();

  // start LIN frame by sending BREAK
  this->_sendBreak();

  // return state machine state
  return this->state;

} // LIN_Master_Base::sendMasterRequestBlocking()



/**
  \brief      Send/receive a blocking LIN slave response frame (no background operation)
  \details    Send/receive a blocking LIN slave response frame (no background operation).
              For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \param[in]  Version   LIN protocol version
  \param[in]  Id        frame idendifier (protected or unprotected)
  \param[in]  NumData   number of data bytes (0..8)
  \param[out] Data      data bytes
  \return     LIN error
*/
LIN_Master_Base::error_t LIN_Master_Base::receiveSlaveResponseBlocking(LIN_Master_Base::version_t Version, uint8_t Id, uint8_t NumData, uint8_t *Data)
{
  // start slave response frame
  this->receiveSlaveResponse(Version, Id, NumData);
  
  // wait until frame is completed
  do
    this->handler();
  while (this->state != LIN_Master_Base::STATE_DONE);

  // copy received data
  this->getFrame(type, id, NumData, Data);

  // return LIN error
  return this->error;

} // LIN_Master_Base::sendMasterRequestBlocking()



/**
  \brief      Handle LIN background operation (call until STATE_DONE is returned)
  \details    Handle LIN background operation (call until STATE_DONE is returned).
              For an explanation of the LIN bus and protocol e.g. see https://en.wikipedia.org/wiki/Local_Interconnect_Network
  \return     LIN state machine state
*/
LIN_Master_Base::state_t LIN_Master_Base::handler(void)
{
  // act according to current state
  switch (this->state)
  {
    // idle or done -> don't do anything
    case LIN_Master_Base::STATE_IDLE:
    case LIN_Master_Base::STATE_DONE:
      break;

    // when sync break done, send rest of frame
    case LIN_Master_Base::STATE_BREAK:
      this->_sendFrame();
      break;

    // when frame done, read and check it
    case LIN_Master_Base::STATE_BODY:
      this->_receiveFrame();
      break;

    // this should never happen..
    default:
      this->error = (LIN_Master_Base::error_t) ((int) this->error | (int) LIN_Master_Base::ERROR_MISC);
      disableTransmitter();
      this->state = LIN_Master_Base::STATE_DONE;

  } // switch (state)
  
  // return state machine state
  return this->state;

} // LIN_Master_Base::handler()

/*-----------------------------------------------------------------------------
    END OF FILE
-----------------------------------------------------------------------------*/
