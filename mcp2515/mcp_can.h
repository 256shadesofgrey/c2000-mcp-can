/****************************************Copyright (c)****************************************************
**                            Shenzhen SeeedStudio Co.,LTD.
**
**                                 http://www.seeedstudio.com
**
**
**
**--------------File Info---------------------------------------------------------------------------------
** File name:                  mcp_can.h
** Latest modified Date:       2015-10-22
** Latest Version:             v0.1
** Descriptions:               mcp2515 main .h file
**
**--------------------------------------------------------------------------------------------------------
** Created by:                 loovee
** Created date:               2012-4-24
** Version:                    v0.1
** Descriptions:
**
**--------------Port Info---------------------------------------------------------------------------------
** Ported by:                  Dmitri Ranfft, Andrew Buckin
** Date of first release:      2015-11-20
** Version:                    v0.1
** Descriptions:               This library was ported to C2000 microcontrollers. Originally it was
**                             written in C++ for Arduino. In case of questions about the original
**                             arduino library, refer to SeedStudio (link is above).
*********************************************************************************************************/
#ifndef _MCP2515_H_
#define _MCP2515_H_

#include "mcp_can_dfs.h"

#define MAX_CHAR_IN_MESSAGE 8

// drivers
#include "sw/drivers/spi/src/32b/f28x/f2802x/spi.h"
#include "sw/drivers/gpio/src/32b/f28x/f2802x/gpio.h"


// **************************************************************************
// modules


// **************************************************************************
// solutions


//!
//! \defgroup DRV8305

//!
//! \ingroup DRV8305
//@{


#ifdef __cplusplus
extern "C" {
#endif

//! \brief Defines the MCP2515 object
//!
typedef struct _MCP2515_Obj_
{
  SPI_Handle       spiHandle;                  //!< the handle for the serial peripheral interface
  GPIO_Handle      gpioHandle;                 //!< the gpio handle that is connected to the mcp2515 enable pin
  GPIO_Number_e    gpio_INT;                   //!< the gpio number that is connected to the mcp2515 interrupt pin
  GPIO_Number_e    gpio_CS;              	   //!< the gpio number that is connected to the mcp2515 CS SPI pin
  bool             canRcvFlag;
} MCP2515_Obj;

//! \brief Defines the MCP2515 handle
//!
typedef struct _MCP2515_Obj_ *MCP2515_Handle;

//MCP2515_Handle mcp2515_handle;

typedef struct _DRV_SPI_MCP2515_Vars_t_
{
  uint16_t                  WriteAddr;
  uint16_t                  ReadAddr;
  uint16_t                  WriteData;
  uint16_t                  ReadData;
}DRV_SPI_MCP2515_Vars_t;


//! \brief     Initializes the MCP2515 object
//! \param[in] pMemory   A pointer to the memory for the MCP2515 object
//! \param[in] numBytes  The number of bytes allocated for the MCP2515 object, bytes
//! \return    The MCP2515 object handle
extern MCP2515_Handle MCP2515_init(void *pMemory,const size_t numBytes);

//! \brief     Sets the SPI handle in the MCP2515
//! \param[in] handle     The MCP2515 handle
//! \param[in] spiHandle  The SPI handle to use
void MCP2515_setSpiHandle(MCP2515_Handle handle,SPI_Handle spiHandle);


//! \brief     Sets the GPIO handle in the MCP2515
//! \param[in] handle       The MCP2515 handle
//! \param[in] gpioHandle   The GPIO handle to use
void MCP2515_setGpioHandle(MCP2515_Handle handle,GPIO_Handle gpioHandle);


//! \brief     Sets the GPIO number in the MCP2515
//! \param[in] handle       The MCP2515 handle
//! \param[in] gpioHandle   The GPIO number to use
void MCP2515_setGpio_INT(MCP2515_Handle handle,GPIO_Number_e gpio_INT);


//! \brief     Sets the GPIO number in the MCP2515
//! \param[in] handle       The MCP2515 handle
//! \param[in] gpioHandle   The GPIO number to use
void MCP2515_setGpio_CS(MCP2515_Handle handle,GPIO_Number_e gpio_CS);


uint16_t MCP2515_spiTransferByte(MCP2515_Handle handle, const uint16_t data);
//void MCP2515_spiWriteByte(MCP2515_Handle handle, const uint16_t data);
//void MCP2515_spiWrite(MCP2515_Handle handle, const uint8_t* data, const uint16_t dataLen);
//uint16_t MCP2515_spiReadByte(MCP2515_Handle handle);
void MCP2515_reset(MCP2515_Handle handle);
uint16_t MCP2515_readRegister(MCP2515_Handle handle, const uint8_t address);
void MCP2515_readRegisterS(MCP2515_Handle handle, const uint8_t address, uint8_t values[], const uint8_t n);
void MCP2515_setRegister(MCP2515_Handle handle,	const uint8_t address, const uint8_t value);
void MCP2515_setRegisterS(MCP2515_Handle handle, const uint8_t address, const uint8_t values[], const uint8_t n);
void MCP2515_modifyRegister(MCP2515_Handle handle, const uint8_t address, const uint8_t mask, const uint8_t data);
INT8U MCP2515_readStatus(MCP2515_Handle handle);
INT8U MCP2515_RX_Status(MCP2515_Handle handle);
INT8U MCP2515_setCANCTRL_Mode(MCP2515_Handle handle, const INT8U newmode);
uint8_t MCP2515_configRate(MCP2515_Handle handle, const uint8_t canSpeed);
void MCP2515_write_id(MCP2515_Handle handle, const uint8_t mcp_addr, const uint8_t ext, const uint8_t id );
void MCP2515_initCANBuffers(MCP2515_Handle handle);
void MCP2515_read_id(MCP2515_Handle handle,	const uint8_t mcp_addr, uint8_t* ext, uint32_t* id );
void MCP2515_write_canMsg(MCP2515_Handle handle, const uint8_t buffer_sidh_addr);
void MCP2515_read_canMsg(MCP2515_Handle handle, const uint8_t buffer_sidh_addr);
void MCP2515_start_transmit(MCP2515_Handle handle, const uint8_t mcp_addr);
INT8U MCP2515_getNextFreeTXBuf(MCP2515_Handle handle, uint8_t *txbuf_n);
uint8_t MCP2515_prepare(MCP2515_Handle handle, const uint8_t canSpeed);
uint8_t MCP2515_begin(MCP2515_Handle handle, uint8_t speedset);
uint8_t MCP2515_init_Mask(MCP2515_Handle handle, uint8_t num, uint8_t ext, uint32_t ulData);
uint8_t MCP2515_init_Filt(MCP2515_Handle handle, uint8_t num, uint8_t ext, uint32_t ulData);
uint8_t MCP2515_setMsg(/*MCP2515_Handle handle, */uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData);
uint8_t MCP2515_clearMsg(MCP2515_Handle handle);
uint8_t MCP2515_sendMsg(MCP2515_Handle handle);
uint8_t MCP2515_sendMsgBuf(MCP2515_Handle handle, uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf);
uint8_t MCP2515_readMsg(MCP2515_Handle handle);
uint8_t MCP2515_readMsgBuf(MCP2515_Handle handle, uint8_t *len, uint8_t buf[]);
uint8_t MCP2515_checkReceive(MCP2515_Handle handle);
uint8_t MCP2515_checkError(MCP2515_Handle handle);
uint32_t MCP2515_getCanId(/*MCP2515_Handle handle*/);
//void MCP2515_setCanId(/*MCP2515_Handle handle,*/ uint32_t id);
void setRcvFlag(MCP2515_Handle handle, bool state);
bool getRcvFlag(MCP2515_Handle handle);

#ifdef __cplusplus
}
#endif

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
