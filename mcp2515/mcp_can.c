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
** Descriptions:               mcp2515 main .c file
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
#include "mcp_can.h"

uint8_t   m_nExtFlg;                                                  /* identifier xxxID             */
																	/* either extended (the 29 LSB) */
																	/* or standard (the 11 LSB)     */
uint32_t  m_nID;                                                      /* can id                       */
uint8_t   m_nDlc;                                                     /* data length:                 */
uint8_t   m_nDta[MAX_CHAR_IN_MESSAGE];                            	/* data                         */
uint8_t   m_nRtr;                                                     /* rtr                          */
uint8_t   m_nfilhit;

MCP2515_Handle MCP2515_init(void *pMemory,const size_t numBytes)
{
	MCP2515_Handle handle;

  if(numBytes < sizeof(MCP2515_Obj))
    return((MCP2515_Handle)NULL);

  // assign the handle
  handle = (MCP2515_Handle)pMemory;

  return(handle);
} // end of MCP2515_init() function

void MCP2515_setSpiHandle(MCP2515_Handle handle,SPI_Handle spiHandle)
{
  MCP2515_Obj *obj = (MCP2515_Obj *)handle;

  // initialize the serial peripheral interface object
  obj->spiHandle = spiHandle;

  return;
} // end of MCP2515_setSpiHandle() function


void MCP2515_setGpioHandle(MCP2515_Handle handle,GPIO_Handle gpioHandle)
{
  MCP2515_Obj *obj = (MCP2515_Obj *)handle;

  // initialize the gpio interface object
  obj->gpioHandle = gpioHandle;

  return;
} // end of MCP2515_setGpioHandle() function


void MCP2515_setGpio_INT(MCP2515_Handle handle,GPIO_Number_e gpio_INT)
{
  MCP2515_Obj *obj = (MCP2515_Obj *)handle;

  // initialize the gpio interface object
  obj->gpio_INT = gpio_INT;

  return;
} // end of MCP2515_setGpio_INT() function


void MCP2515_setGpio_CS(MCP2515_Handle handle,GPIO_Number_e gpio_CS)
{
  MCP2515_Obj *obj = (MCP2515_Obj *)handle;

  // initialize the gpio interface object
  obj->gpio_CS = gpio_CS;

  return;
} // end of MCP2515_setGpio_CS() function

uint16_t MCP2515_spiTransferByte(MCP2515_Handle handle, const uint16_t data)
{
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
//	uint16_t n;
	volatile uint16_t ReadByte;

//	GPIO_setLow(obj->gpioHandle,obj->gpio_CS);

	//SPI_write8(obj->spiHandle, data);
	SPI_write(obj->spiHandle, (data & 0xFF) << 8);

	while(1)
	{
	    if(SPI_getIntFlagStatus(obj->spiHandle)==SPI_IntFlagStatus_Completed)
	        {
	            ReadByte= /*(uint8_t)*/ SPI_read(obj->spiHandle);
	            //ReadByte = SPI_readEmu(obj->spiHandle);
	            //ReadByte=(ReadByte>>(8));
	            break;
	        }
	}


	// wait for registers to update
//	for(n = 0; n < 0x2; n++){            //OK
//		asm(" NOP");
//	}

//	GPIO_setHigh(obj->gpioHandle,obj->gpio_CS);
//	ReadByte= /*(uint8_t)*/ SPI_read(obj->spiHandle);
	return(ReadByte);
}


/*********************************************************************************************************
** Function name:           MCP2515_reset
** Descriptions:            reset the device
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
void MCP2515_reset(MCP2515_Handle handle)
{
	uint16_t n;
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    GPIO_setLow(obj->gpioHandle,obj->gpio_CS);
    MCP2515_spiTransferByte(handle, MCP_RESET);
	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}
    GPIO_setHigh(obj->gpioHandle,obj->gpio_CS);

	for(n = 0; n < 0x30; n++){            //OK
		asm(" NOP");
	}

}

/*********************************************************************************************************
** Function name:           MCP2515_readRegister
** Descriptions:            read register
** input parameters:        address :   register address
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint16_t MCP2515_readRegister(MCP2515_Handle handle, const uint8_t address)
{
	volatile uint16_t ret;
	uint16_t n;
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    GPIO_setLow(obj->gpioHandle,obj->gpio_CS);
    MCP2515_spiTransferByte(handle, MCP_READ);
    MCP2515_spiTransferByte(handle, address);
    MCP2515_spiTransferByte(handle, 0x00);

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle,obj->gpio_CS);
	ret = SPI_read(obj->spiHandle);
	return (ret);
}

/*********************************************************************************************************
** Function name:           MCP2515_readRegisterS
** Descriptions:            read registerS
** input parameters:        address :   register address
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
void MCP2515_readRegisterS(MCP2515_Handle handle, const uint8_t address, uint8_t values[], const uint8_t k)
{
	INT8U i;
	uint16_t n;
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    GPIO_setLow(obj->gpioHandle,obj->gpio_CS);
    MCP2515_spiTransferByte(handle, MCP_READ);
    MCP2515_spiTransferByte(handle, address);
	for (i=0; i<k; i++) {
		MCP2515_spiTransferByte(handle, 0x00);

		for(n = 0; n < 0xf; n++){            //OK
			asm(" NOP");
		}

		values[i] = SPI_read(obj->spiHandle);
	}
	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle,obj->gpio_CS);

}

/*********************************************************************************************************
** Function name:           MCP2515_setRegister
** Descriptions:            set register
** input parameters:        address :   register address
                            value   :   register value
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
void MCP2515_setRegister(MCP2515_Handle handle, const uint8_t address, const uint8_t value)
{
	uint16_t n;
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    GPIO_setLow(obj->gpioHandle,obj->gpio_CS);
    MCP2515_spiTransferByte(handle, MCP_WRITE);
    MCP2515_spiTransferByte(handle, address);
    MCP2515_spiTransferByte(handle, value);

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle,obj->gpio_CS);

}

/*********************************************************************************************************
** Function name:           MCP2515_setRegisterS
** Descriptions:            set registerS
** input parameters:        address : first register 's address
                            values[]: value
                            n       : data length
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
void MCP2515_setRegisterS(MCP2515_Handle handle, const uint8_t address, const uint8_t values[], const uint8_t k)
{
	INT8U i;
	uint16_t n;
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    GPIO_setLow(obj->gpioHandle,obj->gpio_CS);
    MCP2515_spiTransferByte(handle, MCP_WRITE);
    MCP2515_spiTransferByte(handle, address);

	for (i=0; i<k; i++) {
		MCP2515_spiTransferByte(handle, values[i]);
//		for(n = 0; n < 0xf; n++){            //OK
//			asm(" NOP");
//		}
	}
	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle,obj->gpio_CS);
}

/*********************************************************************************************************
** Function name:           MCP2515_modifyRegister
** Descriptions:            set bit of one register
** input parameters:        address : register address
                            mask    : bit address
                            data    : data
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
/* set bit of one register      */
void MCP2515_modifyRegister(MCP2515_Handle handle, const uint8_t address, const uint8_t mask, const uint8_t data)
{

	uint16_t n;
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    GPIO_setLow(obj->gpioHandle,obj->gpio_CS);
    MCP2515_spiTransferByte(handle, MCP_BITMOD);
    MCP2515_spiTransferByte(handle, address);
    MCP2515_spiTransferByte(handle, mask);
    MCP2515_spiTransferByte(handle, data);

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle,obj->gpio_CS);

}

/*********************************************************************************************************
** Function name:           MCP2515_readStatus
** Descriptions:            read MCP2515's Status
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          the device's status
*********************************************************************************************************/
INT8U MCP2515_readStatus(MCP2515_Handle handle)
{

	uint16_t ret;
	uint16_t n;
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    GPIO_setLow(obj->gpioHandle,obj->gpio_CS);
    MCP2515_spiTransferByte(handle, MCP_READ_STATUS);
    MCP2515_spiTransferByte(handle, 0x00);

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle,obj->gpio_CS);
	ret = SPI_read(obj->spiHandle);
	return (ret);
}
/*********************************************************************************************************
** Function name:           MCP2515_RX_Status
** Descriptions:            read MCP2515's Status
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          the device's status
*********************************************************************************************************/
INT8U MCP2515_RX_Status(MCP2515_Handle handle)
{

	uint16_t ret;
	uint16_t n;
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
    GPIO_setLow(obj->gpioHandle,obj->gpio_CS);
    MCP2515_spiTransferByte(handle, MCP_RX_STATUS);
    MCP2515_spiTransferByte(handle, 0x00);

	for(n = 0; n < 0xf; n++){            //OK
		asm(" NOP");
	}

	GPIO_setHigh(obj->gpioHandle,obj->gpio_CS);
	ret = SPI_read(obj->spiHandle);
	return (ret);
}

/*********************************************************************************************************
** Function name:           MCP2515_setCANCTRL_Mode
** Descriptions:            set control mode
** input parameters:        newmode : mode
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
INT8U MCP2515_setCANCTRL_Mode(MCP2515_Handle handle, const INT8U newmode)
{
    INT8U i;
    //INT8U n;

    MCP2515_modifyRegister(handle, MCP_CANCTRL, MODE_MASK, newmode);

//	for(n = 0; n < 0xf; n++){            //OK
//		asm(" NOP");
//	}

    i = MCP2515_readRegister(handle, MCP_CANCTRL);
    i &= MODE_MASK;

    if ( i == newmode ) {
        return MCP2515_OK;
    }
    else {
        return MCP2515_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           MCP2515_configRate
** Descriptions:            set boadrate
** input parameters:        canSpeed    : boadrate
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_configRate(MCP2515_Handle handle, const uint8_t canSpeed)
{
    uint8_t set, cfg1, cfg2, cfg3;
    set = 1;
    switch (canSpeed) {
        case (CAN_5KBPS):
        cfg1 = MCP_16MHz_5kBPS_CFG1;
        cfg2 = MCP_16MHz_5kBPS_CFG2;
        cfg3 = MCP_16MHz_5kBPS_CFG3;
        break;

        case (CAN_10KBPS):
        cfg1 = MCP_16MHz_10kBPS_CFG1;
        cfg2 = MCP_16MHz_10kBPS_CFG2;
        cfg3 = MCP_16MHz_10kBPS_CFG3;
        break;

        case (CAN_20KBPS):
        cfg1 = MCP_16MHz_20kBPS_CFG1;
        cfg2 = MCP_16MHz_20kBPS_CFG2;
        cfg3 = MCP_16MHz_20kBPS_CFG3;
        break;

        case (CAN_40KBPS):
        cfg1 = MCP_16MHz_40kBPS_CFG1;
        cfg2 = MCP_16MHz_40kBPS_CFG2;
        cfg3 = MCP_16MHz_40kBPS_CFG3;
        break;

        case (CAN_50KBPS):
        cfg1 = MCP_16MHz_50kBPS_CFG1;
        cfg2 = MCP_16MHz_50kBPS_CFG2;
        cfg3 = MCP_16MHz_50kBPS_CFG3;
        break;

        case (CAN_80KBPS):
        cfg1 = MCP_16MHz_80kBPS_CFG1;
        cfg2 = MCP_16MHz_80kBPS_CFG2;
        cfg3 = MCP_16MHz_80kBPS_CFG3;
        break;

        case (CAN_100KBPS):                                             /* 100KBPS                  */
        cfg1 = MCP_16MHz_100kBPS_CFG1;
        cfg2 = MCP_16MHz_100kBPS_CFG2;
        cfg3 = MCP_16MHz_100kBPS_CFG3;
        break;

        case (CAN_125KBPS):
        cfg1 = MCP_16MHz_125kBPS_CFG1;
        cfg2 = MCP_16MHz_125kBPS_CFG2;
        cfg3 = MCP_16MHz_125kBPS_CFG3;
        break;

        case (CAN_200KBPS):
        cfg1 = MCP_16MHz_200kBPS_CFG1;
        cfg2 = MCP_16MHz_200kBPS_CFG2;
        cfg3 = MCP_16MHz_200kBPS_CFG3;
        break;

        case (CAN_250KBPS):
        cfg1 = MCP_16MHz_250kBPS_CFG1;
        cfg2 = MCP_16MHz_250kBPS_CFG2;
        cfg3 = MCP_16MHz_250kBPS_CFG3;
        break;

        case (CAN_500KBPS):
        cfg1 = MCP_16MHz_500kBPS_CFG1;
        cfg2 = MCP_16MHz_500kBPS_CFG2;
        cfg3 = MCP_16MHz_500kBPS_CFG3;
        break;

        default:
        set = 0;
        break;
    }

    if (set) {
        MCP2515_setRegister(handle, MCP_CNF1, cfg1);
        MCP2515_setRegister(handle, MCP_CNF2, cfg2);
        MCP2515_setRegister(handle, MCP_CNF3, cfg3);
        return MCP2515_OK;
    }
    else {
        return MCP2515_FAIL;
    }
}

/*********************************************************************************************************
** Function name:           MCP2515_write_id
** Descriptions:            write can id
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
void MCP2515_write_id(MCP2515_Handle handle,
								const uint8_t mcp_addr,
                                const uint8_t ext,
                                const uint8_t id )
{
    uint16_t canid;
    uint8_t tbufdata[4];

    canid = (uint16_t)(id & 0x0FFFF);

    if ( ext == 1) {
        tbufdata[MCP_EID0] = (INT8U) (canid & 0xFF);
        tbufdata[MCP_EID8] = (INT8U) (canid / 256);
        canid = (uint16_t)( id / 0x10000L );
        tbufdata[MCP_SIDL] = (INT8U) (canid & 0x03);
        tbufdata[MCP_SIDL] += (INT8U) ((canid & 0x1C )*8);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (INT8U) (canid / 32 );
    }
    else {
        tbufdata[MCP_SIDH] = (INT8U) (canid / 8 );
        tbufdata[MCP_SIDL] = (INT8U) ((canid & 0x07 )<<5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
    MCP2515_setRegisterS(handle, mcp_addr, tbufdata, 4 );
}

/*********************************************************************************************************
** Function name:           MCP2515_initCANBuffers
** Descriptions:            init canbuffers
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
void MCP2515_initCANBuffers(MCP2515_Handle handle)
{
    uint8_t i, a1, a2, a3;

    uint8_t ext = 1;
    uint32_t ulMask = 0x00, ulFilt = 0x00;


    MCP2515_write_id(handle, MCP_RXM0SIDH, ext, ulMask);
    MCP2515_write_id(handle, MCP_RXM1SIDH, ext, ulMask);
                                                                        /* Anyway, set all filters to 0 */
                                                                        /* :                            */
    MCP2515_write_id(handle, MCP_RXF0SIDH, ext, ulFilt);                        /* RXB0: extended               */
    MCP2515_write_id(handle, MCP_RXF1SIDH, ext, ulFilt);                        /* AND standard                 */
    MCP2515_write_id(handle, MCP_RXF2SIDH, ext, ulFilt);                        /* RXB1: extended               */
    MCP2515_write_id(handle, MCP_RXF3SIDH, ext, ulFilt);                        /* AND standard                 */
    MCP2515_write_id(handle, MCP_RXF4SIDH, ext, ulFilt);
    MCP2515_write_id(handle, MCP_RXF5SIDH, ext, ulFilt);

                                                                        /* Clear, deactivate the three  */
                                                                        /* transmit buffers             */
                                                                        /* TXBnCTRL -> TXBnD7           */
    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++) {                                          /* in-buffer loop               */
        MCP2515_setRegister(handle, a1, 0);
        MCP2515_setRegister(handle, a2, 0);
        MCP2515_setRegister(handle, a3, 0);
        a1++;
        a2++;
        a3++;
    }
    MCP2515_setRegister(handle, MCP_RXB0CTRL, 0);
    MCP2515_setRegister(handle, MCP_RXB1CTRL, 0);
}

/*********************************************************************************************************
** Function name:           MCP2515_read_id
** Descriptions:            read can id
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
void MCP2515_read_id(MCP2515_Handle handle,
									const uint8_t mcp_addr,
                                     uint8_t* ext,
                                     uint32_t* id )
{
    uint8_t tbufdata[4];

    *ext = 0;
    *id = 0;

    MCP2515_readRegisterS(handle, mcp_addr, tbufdata, 4 );

    *id = (tbufdata[MCP_SIDH]<<3) + (tbufdata[MCP_SIDL]>>5);

    if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M ) {
                                                                        /* extended id                  */
        *id = (*id<<2) + (tbufdata[MCP_SIDL] & 0x03);
        *id <<= 16;
        *id = *id +(tbufdata[MCP_EID8]<<8) + tbufdata[MCP_EID0];
        *ext = 1;
    }
}

/*********************************************************************************************************
** Function name:           MCP2515_write_canMsg
** Descriptions:            write msg
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
void MCP2515_write_canMsg(MCP2515_Handle handle, const uint8_t buffer_sidh_addr)
{
    uint8_t mcp_addr;
    mcp_addr = buffer_sidh_addr;
    MCP2515_setRegisterS(handle, mcp_addr+5, m_nDta, m_nDlc );                  /* write data bytes             */
    if ( m_nRtr == 1)  m_nDlc |= MCP_RTR_MASK;                          /* if RTR set bit in byte       */
    MCP2515_setRegister(handle, (mcp_addr+4), m_nDlc );                        /* write the RTR and DLC        */
    MCP2515_write_id(handle, mcp_addr, m_nExtFlg, m_nID );                 /* write CAN id                 */
}

/*********************************************************************************************************
** Function name:           MCP2515_read_canMsg
** Descriptions:            read message
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
void MCP2515_read_canMsg(MCP2515_Handle handle, const uint8_t buffer_sidh_addr)        /* read can msg                 */
{
    uint8_t mcp_addr, ctrl;

    mcp_addr = buffer_sidh_addr;

    MCP2515_read_id(handle, mcp_addr, &m_nExtFlg,&m_nID );

    ctrl = MCP2515_readRegister(handle, mcp_addr-1 );
    m_nDlc = MCP2515_readRegister(handle, mcp_addr+4 );

    if ((ctrl & 0x08)) {
        m_nRtr = 1;
    }
    else {
        m_nRtr = 0;
    }

    m_nDlc &= MCP_DLC_MASK;
    MCP2515_readRegisterS(handle, mcp_addr+5, &(m_nDta[0]), m_nDlc );
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
void MCP2515_start_transmit(MCP2515_Handle handle, const uint8_t mcp_addr)              /* start transmit               */
{
    MCP2515_modifyRegister(handle, mcp_addr-1 , MCP_TXB_TXREQ_M, MCP_TXB_TXREQ_M );
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
INT8U MCP2515_getNextFreeTXBuf(MCP2515_Handle handle, uint8_t *txbuf_n)                 /* get Next free txbuf          */
{
    uint8_t res, i, ctrlval;
    uint8_t ctrlregs[MCP_N_TXBUFFERS] = { MCP_TXB0CTRL, MCP_TXB1CTRL, MCP_TXB2CTRL };

    res = MCP_ALLTXBUSY;
    *txbuf_n = 0x00;

                                                                        /* check all 3 TX-Buffers       */
    for (i=0; i<MCP_N_TXBUFFERS; i++) {
        ctrlval = MCP2515_readRegister(handle, ctrlregs[i] );
        if ( (ctrlval & MCP_TXB_TXREQ_M) == 0 ) {
            *txbuf_n = ctrlregs[i]+1;                                   /* return SIDH-address of Buffe */
                                                                        /* r                            */
            res = MCP2515_OK;
            return res;                                                 /* ! function exit              */
        }
    }
    return res;
}

/*********************************************************************************************************
** Function name:           MCP2515_init
** Descriptions:            init the device
** input parameters:        canSpeed    : boadrate
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_prepare(MCP2515_Handle handle, const uint8_t canSpeed)                       /* MCP2515init                  */
{

	uint8_t res;

    MCP2515_reset(handle);

#ifdef MCP2515_My_init

//	MCP2515_setRegister(handle, MCP_CNF1,MCP_16MHz_500kBPS_CFG1);
//	MCP2515_setRegister(handle, MCP_CNF2,MCP_16MHz_500kBPS_CFG2);
//	MCP2515_setRegister(handle, MCP_CNF3,MCP_16MHz_500kBPS_CFG3);
    MCP2515_configRate(handle, canSpeed);

    MCP2515_setRegister(handle, MCP_CANINTE, MCP_RX0IE | MCP_RX1IE);
    MCP2515_setRegister(handle, MCP_RXB0CTRL, MCP_RXM1 | MCP_RXM0);
    MCP2515_setRegister(handle, MCP_RXB1CTRL, MCP_RXM1 | MCP_RXM0);

    MCP2515_setRegister(handle, MCP_RXM0SIDH, 0);
    MCP2515_setRegister(handle, MCP_RXM0SIDL, 0);
    MCP2515_setRegister(handle, MCP_RXM0EID8, 0);
    MCP2515_setRegister(handle, MCP_RXM0EID0, 0);

    MCP2515_setRegister(handle, MCP_RXM1SIDH, 0);
    MCP2515_setRegister(handle, MCP_RXM1SIDL, 0);
    MCP2515_setRegister(handle, MCP_RXM1EID8, 0);
    MCP2515_setRegister(handle, MCP_RXM1EID0, 0);


    MCP2515_setRegister(handle, MCP_BFPCTRL, 0);
    MCP2515_setRegister(handle, MCP_TXRTCTRL, 0);

    MCP2515_modifyRegister(handle, MCP_CANCTRL, 0xE0, 0);
    res= MCP2515_readStatus(handle);

#else
    res = MCP2515_setCANCTRL_Mode(handle, MODE_CONFIG);
    if(res > 0)
    {
#if DEBUG_MODE
      Serial.print("Enter setting mode fall\r\n");
#endif
      return res;
    }
#if DEBUG_MODE
    Serial.print("Enter setting mode success \r\n");
#endif

                                                                        /* set boadrate                 */
    if(MCP2515_configRate(handle, canSpeed))
    {
#if DEBUG_MODE
      Serial.print("set rate fall!!\r\n");
#endif
      return res;
    }
#if DEBUG_MODE
    Serial.print("set rate success!!\r\n");
#endif

    if ( res == MCP2515_OK ) {

                                                                        /* init canbuffers              */
        MCP2515_initCANBuffers(handle);

                                                                        /* interrupt mode               */
        MCP2515_setRegister(handle, MCP_CANINTE, MCP_RX0IE | MCP_RX1IE);

#if (DEBUG_RXANY==1)
                                                                        /* enable both receive-buffers  */
                                                                        /* to receive any message       */
                                                                        /* and enable rollover          */
        MCP2515_modifyRegister(MCP2515_Handle handle, MCP_RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_ANY | MCP_RXB_BUKT_MASK);
        MCP2515_modifyRegister(MCP2515_Handle handle, MCP_RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_ANY);
#else
                                                                        /* enable both receive-buffers  */
                                                                        /* to receive messages          */
                                                                        /* with std. and ext. identifie */
                                                                        /* rs                           */
                                                                        /* and enable rollover          */
        MCP2515_modifyRegister(handle, MCP_RXB0CTRL,
        MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
        MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK );
        MCP2515_modifyRegister(handle, MCP_RXB1CTRL, MCP_RXB_RX_MASK,
        MCP_RXB_RX_STDEXT);
#endif
                                                                        /* enter normal mode            */
        res = MCP2515_setCANCTRL_Mode(handle, MODE_NORMAL);
        if(res)
        {
#if DEBUG_MODE
          Serial.print("Enter Normal Mode Fall!!\r\n");
#endif
          return res;
        }


#if DEBUG_MODE
          Serial.print("Enter Normal Mode Success!!\r\n");
#endif

    }
#endif
    return res;
}

/*********************************************************************************************************
** Function name:           init
** Descriptions:            init can and set speed
** input parameters:        speedset : can boadrate
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_begin(MCP2515_Handle handle, uint8_t speedset)
{
    uint8_t res;

    setRcvFlag(handle, false);
    //mcp2515_handle = handle;
    res = MCP2515_prepare(handle, speedset);
    if (res == MCP2515_OK) return CAN_OK;
    else return CAN_FAILINIT;
}

/*********************************************************************************************************
** Function name:           init_Mask
** Descriptions:            init canid Masks
** input parameters:        num :   Mask number
                            ext :   if ext id
                            ulData: data
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_init_Mask(MCP2515_Handle handle, uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
#if DEBUG_MODE
    Serial.print("Begin to set Mask!!\r\n");
#endif
    res = MCP2515_setCANCTRL_Mode(handle, MODE_CONFIG);
    if(res > 0){
#if DEBUG_MODE
    Serial.print("Enter setting mode fall\r\n");
#endif
  return res;
}

    if (num == 0){
        MCP2515_write_id(handle, MCP_RXM0SIDH, ext, ulData);

    }
    else if(num == 1){
        MCP2515_write_id(handle, MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP2515_FAIL;

    res = MCP2515_setCANCTRL_Mode(handle, MODE_NORMAL);
    if(res > 0){
#if DEBUG_MODE
    Serial.print("Enter normal mode fall\r\n");
#endif
    return res;
  }
#if DEBUG_MODE
    Serial.print("set Mask success!!\r\n");
#endif
    return res;
}

/*********************************************************************************************************
** Function name:           init_Filt
** Descriptions:            init canid filters
** input parameters:        num  :  Filters number
                            ext  :  if ext id
                            ulData: data
** Output parameters:       NONE
** Returned value:          ifsuccess
*********************************************************************************************************/
uint8_t MCP2515_init_Filt(MCP2515_Handle handle, uint8_t num, uint8_t ext, uint32_t ulData)
{
    uint8_t res = MCP2515_OK;
#if DEBUG_MODE
    Serial.print("Begin to set Filter!!\r\n");
#endif
    res = MCP2515_setCANCTRL_Mode(handle, MODE_CONFIG);
    if(res > 0)
    {
#if DEBUG_MODE
      Serial.print("Enter setting mode fall\r\n");
#endif
      return res;
    }

    switch( num )
    {
        case 0:
        MCP2515_write_id(handle, MCP_RXF0SIDH, ext, ulData);
        break;

        case 1:
        MCP2515_write_id(handle, MCP_RXF1SIDH, ext, ulData);
        break;

        case 2:
        MCP2515_write_id(handle, MCP_RXF2SIDH, ext, ulData);
        break;

        case 3:
        MCP2515_write_id(handle, MCP_RXF3SIDH, ext, ulData);
        break;

        case 4:
        MCP2515_write_id(handle, MCP_RXF4SIDH, ext, ulData);
        break;

        case 5:
        MCP2515_write_id(handle, MCP_RXF5SIDH, ext, ulData);
        break;

        default:
        res = MCP2515_FAIL;
    }

    res = MCP2515_setCANCTRL_Mode(handle, MODE_NORMAL);
    if(res > 0)
    {
#if DEBUG_MODE
      Serial.print("Enter normal mode fall\r\nSet filter fail!!\r\n");
#endif
      return res;
    }
#if DEBUG_MODE
    Serial.print("set Filter success!!\r\n");
#endif

    return res;
}

/*********************************************************************************************************
** Function name:           setMsg
** Descriptions:            set can message, such as dlc, id, dta[] and so on
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_setMsg(/*MCP2515_Handle handle,*/uint32_t id, uint8_t ext, uint8_t len, uint8_t *pData)
{
    int i = 0;
    m_nExtFlg = ext;
    m_nID     = id;
    m_nDlc    = len;
    for(i = 0; i<MAX_CHAR_IN_MESSAGE; i++)
    	m_nDta[i] = *(pData+i);
    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           clearMsg
** Descriptions:            set all message to zero
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_clearMsg(MCP2515_Handle handle)
{
    m_nID       = 0;
    m_nDlc      = 0;
    m_nExtFlg   = 0;
    m_nRtr      = 0;
    m_nfilhit   = 0;
    int i;
    for(i = 0; i<m_nDlc; i++ )
      m_nDta[i] = 0x00;

    return MCP2515_OK;
}

/*********************************************************************************************************
** Function name:           sendMsg
** Descriptions:            send message
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_sendMsg(MCP2515_Handle handle)
{
    uint8_t res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;

    do {
        res = MCP2515_getNextFreeTXBuf(handle, &txbuf_n);                       /* info = addr.                 */
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if(uiTimeOut == TIMEOUTVALUE) return CAN_GETTXBFTIMEOUT;            /* get tx buff time out         */
    uiTimeOut = 0;
    MCP2515_write_canMsg(handle, txbuf_n);
    MCP2515_start_transmit(handle, txbuf_n);
    do
    {
        uiTimeOut++;
        res1= MCP2515_readRegister(handle, txbuf_n);  			/* read send buff ctrl reg 	*/ //<---------------------------------------------------------- res1= CAN.MCP2515_readRegister(txbuf_n);
        res1 = res1 & 0x08;
    }while(res1 && (uiTimeOut < TIMEOUTVALUE));
    if(uiTimeOut == TIMEOUTVALUE) return CAN_SENDMSGTIMEOUT;            /* send msg timeout             */
    return CAN_OK;

}

/*********************************************************************************************************
** Function name:           sendMsgBuf
** Descriptions:            send buf
** input parameters:        id  :   can id
                            ext :   if ext
                            len :   buf length
                            *buf:   data buf
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_sendMsgBuf(MCP2515_Handle handle, uint32_t id, uint8_t ext, uint8_t len, uint8_t *buf)
{
	MCP2515_setMsg(id, ext, len, buf);
    return MCP2515_sendMsg(handle);
}

/*********************************************************************************************************
** Function name:           readMsg
** Descriptions:            read message
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_readMsg(MCP2515_Handle handle)
{
    uint8_t stat, res;

    stat = MCP2515_readStatus(handle);

    if ( stat & MCP_STAT_RX0IF ) {                                      /* Msg in Buffer 0              */
        MCP2515_read_canMsg(handle, MCP_RXBUF_0);
        MCP2515_modifyRegister(handle, MCP_CANINTF, MCP_RX0IF, 0);
        res = CAN_OK;
    }
    else if ( stat & MCP_STAT_RX1IF ) {                                 /* Msg in Buffer 1              */
        MCP2515_read_canMsg(handle, MCP_RXBUF_1);
        MCP2515_modifyRegister(handle, MCP_CANINTF, MCP_RX1IF, 0);
        res = CAN_OK;
    }
    else {
        res = CAN_NOMSG;
    }
    return res;
}

/*********************************************************************************************************
** Function name:           readMsgBuf
** Descriptions:            read message buf
** input parameters:        len:    data length
** Output parameters:       buf:    data buf
** Returned value:          if success
*********************************************************************************************************/
uint8_t MCP2515_readMsgBuf(MCP2515_Handle handle, uint8_t *len, uint8_t buf[])
{
    int res = MCP2515_readMsg(handle);

    if (res != CAN_OK)
    {
        *len = 0;
        return res;
    }

    *len = m_nDlc;
    int i;
    for(i = 0; i < m_nDlc; i++)
    {
        buf[i] = m_nDta[i];
    }

    setRcvFlag(handle, false);
    return res;
}

/*********************************************************************************************************
** Function name:           checkReceive
** Descriptions:            check if got something
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          if something
*********************************************************************************************************/
uint8_t MCP2515_checkReceive(MCP2515_Handle handle)
{
    INT8U res;
    res = MCP2515_readStatus(handle);                                         /* RXnIF in Bit 1 and 0         */
    if ( res & MCP_STAT_RXIF_MASK ) {
        return CAN_MSGAVAIL;
    }
    else {
        return CAN_NOMSG;
    }
}

/*********************************************************************************************************
** Function name:           checkError
** Descriptions:            if something error
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
uint8_t MCP2515_checkError(MCP2515_Handle handle)
{
    uint8_t eflg = MCP2515_readRegister(handle, MCP_EFLG);

    if ( eflg & MCP_EFLG_ERRORMASK ) {
        return CAN_CTRLERROR;
    }
    else {
        return CAN_OK;
    }
}

/*********************************************************************************************************
** Function name:           MCP2515_getCanId
** Descriptions:            when receive something ,u can get the can id!!
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
uint32_t MCP2515_getCanId(/*MCP2515_Handle handle*/)
{
  return m_nID;
}

/*********************************************************************************************************
** Function name:           MCP2515_setCanId
** Descriptions:
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
//void MCP2515_setCanId(/*MCP2515_Handle handle,*/ uint32_t id)
//{
//  m_nID = id;
//}

/*********************************************************************************************************
** Function name:           setRcvFlag
** Descriptions:            sets canRcvFlag to the given state (use to set to false after reading the buffer)
** input parameters:        true/false
** Output parameters:       NONE
** Returned value:          NONE
*********************************************************************************************************/
void setRcvFlag(MCP2515_Handle handle, bool state){
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
	obj->canRcvFlag = state;
}

/*********************************************************************************************************
** Function name:           getRcvFlag
** Descriptions:            returns canRcvFlag state
** input parameters:        NONE
** Output parameters:       NONE
** Returned value:          true/false
*********************************************************************************************************/
bool getRcvFlag(MCP2515_Handle handle){
	MCP2515_Obj *obj = (MCP2515_Obj *)handle;
	return obj->canRcvFlag;
}
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
