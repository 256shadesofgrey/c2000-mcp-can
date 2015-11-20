/****************************************Copyright (c)****************************************************
**                            Shenzhen SeeedStudio Co.,LTD.
**
**                                 http://www.seeedstudio.com
**
**
**
**--------------File Info---------------------------------------------------------------------------------
** File name:                  mcp_can_dfs.h
** Latest modified Date:       2015-10-22
** Latest Version:             v0.1
** Descriptions:               mcp2515 defineS file
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
#ifndef _MCP2515DFS_H_
#define _MCP2515DFS_H_

#ifndef INT32U
#define INT32U unsigned long
#endif

#ifndef INT8U
#define INT8U unsigned char
#endif

#ifndef uint8_t
#define uint8_t unsigned char
#endif

// if print debug information
#ifndef DEBUG_MODE
#define DEBUG_MODE 0
#endif

/*
 *   Begin mt
 */
#define TIMEOUTVALUE    50
#define MCP_SIDH        0
#define MCP_SIDL        1
#define MCP_EID8        2
#define MCP_EID0        3

#define MCP_TXB_EXIDE_M     0x08                                        /* In TXBnSIDL                  */
#define MCP_DLC_MASK        0x0F                                        /* 4 LSBits                     */
#define MCP_RTR_MASK        0x40                                        /* (1<<6) Bit 6                 */

#define MCP_RXB_RX_ANY      0x60
#define MCP_RXB_RX_EXT      0x40
#define MCP_RXB_RX_STD      0x20
#define MCP_RXB_RX_STDEXT   0x00
#define MCP_RXB_RX_MASK     0x60
#define MCP_RXB_BUKT_MASK   (1<<2)

/*
** Bits in the TXBnCTRL registers.
*/
#define MCP_TXB_TXBUFE_M    0x80
#define MCP_TXB_ABTF_M      0x40
#define MCP_TXB_MLOA_M      0x20
#define MCP_TXB_TXERR_M     0x10
#define MCP_TXB_TXREQ_M     0x08
#define MCP_TXB_TXIE_M      0x04
#define MCP_TXB_TXP10_M     0x03

#define MCP_TXB_RTR_M       0x40                                        /* In TXBnDLC                   */
#define MCP_RXB_IDE_M       0x08                                        /* In RXBnSIDL                  */
#define MCP_RXB_RTR_M       0x40                                        /* In RXBnDLC                   */

#define MCP_STAT_RXIF_MASK   (0x03)
#define MCP_STAT_RX0IF (1<<0)
#define MCP_STAT_RX1IF (1<<1)

#define MCP_EFLG_RX1OVR (1<<7)
#define MCP_EFLG_RX0OVR (1<<6)
#define MCP_EFLG_TXBO   (1<<5)
#define MCP_EFLG_TXEP   (1<<4)
#define MCP_EFLG_RXEP   (1<<3)
#define MCP_EFLG_TXWAR  (1<<2)
#define MCP_EFLG_RXWAR  (1<<1)
#define MCP_EFLG_EWARN  (1<<0)
#define MCP_EFLG_ERRORMASK  (0xF8)                                      /* 5 MS-Bits                    */


/*
 *   Define MCP2515 register addresses
 */

#define MCP_RXF0SIDH    0x00
#define MCP_RXF0SIDL    0x01
#define MCP_RXF0EID8    0x02
#define MCP_RXF0EID0    0x03
#define MCP_RXF1SIDH    0x04
#define MCP_RXF1SIDL    0x05
#define MCP_RXF1EID8    0x06
#define MCP_RXF1EID0    0x07
#define MCP_RXF2SIDH    0x08
#define MCP_RXF2SIDL    0x09
#define MCP_RXF2EID8    0x0A
#define MCP_RXF2EID0    0x0B
#define MCP_BFPCTRL     	0x0C
#define MCP_TXRTCTRL    	0x0D
#define MCP_CANSTAT     0x0E
#define MCP_CANCTRL     0x0F
#define MCP_RXF3SIDH    0x10
#define MCP_RXF3SIDL    0x11
#define MCP_RXF3EID8    0x12
#define MCP_RXF3EID0    0x13
#define MCP_RXF4SIDH    0x14
#define MCP_RXF4SIDL    0x15
#define MCP_RXF4EID8    0x16
#define MCP_RXF4EID0    0x17
#define MCP_RXF5SIDH    0x18
#define MCP_RXF5SIDL    0x19
#define MCP_RXF5EID8    0x1A
#define MCP_RXF5EID0    0x1B
#define MCP_TEC            0x1C
#define MCP_REC            0x1D //0-1d
#define MCP_RXM0SIDH    0x20
#define MCP_RXM0SIDL    0x21
#define MCP_RXM0EID8    0x22
#define MCP_RXM0EID0    0x23
#define MCP_RXM1SIDH    0x24
#define MCP_RXM1SIDL    0x25
#define MCP_RXM1EID8    0x26
#define MCP_RXM1EID0    0x27
#define MCP_CNF3        0x28
#define MCP_CNF2        0x29
#define MCP_CNF1        0x2A
#define MCP_CANINTE        0x2B
#define MCP_CANINTF        0x2C
#define MCP_EFLG        0x2D //20-2d
#define MCP_TXB0CTRL    0x30
#define MCP_TXB1CTRL    0x40
#define MCP_TXB2CTRL    0x50
#define MCP_RXB0CTRL    0x60
#define MCP_RXB0SIDH    0x61
#define MCP_RXB1CTRL    0x70
#define MCP_RXB1SIDH    0x71
#define MCP_BFPCTRL     0x0C

#define MCP_TX_INT        0x1C                                          /* Enable all transmit interrup */
                                                                        /* ts                           */
#define MCP_TX01_INT    0x0C                                            /* Enable TXB0 and TXB1 interru */
                                                                        /* pts                          */
#define MCP_RX_INT        0x03                                          /* Enable receive interrupts    */
#define MCP_NO_INT        0x00                                          /* Disable all interrupts       */

#define MCP_TX01_MASK    0x14
#define MCP_TX_MASK        0x54

/*
 *   Define SPI Instruction Set
 */

#define MCP_WRITE        0x02

#define MCP_READ        0x03

#define MCP_BITMOD        0x05

#define MCP_LOAD_TX0    0x40
#define MCP_LOAD_TX1    0x42
#define MCP_LOAD_TX2    0x44

#define MCP_RTS_TX0        0x81
#define MCP_RTS_TX1        0x82
#define MCP_RTS_TX2        0x84
#define MCP_RTS_ALL        0x87

#define MCP_READ_RX0    0x90
#define MCP_READ_RX1    0x94

#define MCP_READ_STATUS    0xA0

#define MCP_RX_STATUS    0xB0

#define MCP_RESET        0xC0


/*
 *   CANCTRL Register Values
 */

#define MODE_NORMAL     0x00
#define MODE_SLEEP      0x20
#define MODE_LOOPBACK   0x40
#define MODE_LISTENONLY 0x60
#define MODE_CONFIG     0x80
#define MODE_POWERUP    0xE0
#define MODE_MASK        0xE0
#define ABORT_TX        0x10
#define MODE_ONESHOT    0x08
#define CLKOUT_ENABLE    0x04
#define CLKOUT_DISABLE    0x00
#define CLKOUT_PS1        0x00
#define CLKOUT_PS2        0x01
#define CLKOUT_PS4        0x02
#define CLKOUT_PS8        0x03


/*
 *   CNF1 Register Values
 */

#define SJW1            0x00
#define SJW2            0x40
#define SJW3            0x80
#define SJW4            0xC0


/*
 *   CNF2 Register Values
 */

#define BTLMODE            0x80
#define SAMPLE_1X       0x00
#define SAMPLE_3X       0x40


/*
 *   CNF3 Register Values
 */

#define SOF_ENABLE        0x80
#define SOF_DISABLE        0x00
#define WAKFIL_ENABLE    0x40
#define WAKFIL_DISABLE    0x00


/*
 *   CANINTF Register Bits
 */

#define MCP_RX0IF        0x01
#define MCP_RX1IF        0x02
#define MCP_TX0IF        0x04
#define MCP_TX1IF        0x08
#define MCP_TX2IF        0x10
#define MCP_ERRIF        0x20
#define MCP_WAKIF        0x40
#define MCP_MERRF        0x80

/*
 *   CANINTE Register Bits
 */

#define MCP_RX0IE        0x01
#define MCP_RX1IE        0x02
#define MCP_TX0IE        0x04
#define MCP_TX1IE        0x08
#define MCP_TX2IE        0x10
#define MCP_ERRIE        0x20
#define MCP_WAKIE        0x40
#define MCP_MERRE        0x80

/*
 *   RXB0CTRL Register Bits
 */

#define MCP_FILHIT0      0x01
#define MCP_BUKT1        0x02
#define MCP_BUKT2        0x04
#define MCP_RXRTR        0x08
#define MCP_RXM0         0x20
#define MCP_RXM1         0x40

/*
 *   RXB1CTRL Register Bits
 */

#define MCP_FILHIT0      0x01
#define MCP_FILHIT1      0x02
#define MCP_FILHIT2      0x04
#define MCP_RXRTR        0x08
#define MCP_RXM0         0x20
#define MCP_RSM1         0x40

/*
 *  speed 16M
 */
#define MCP_16MHz_500kBPS_CFG1 (0x00)
#define MCP_16MHz_500kBPS_CFG2 (0xb0)
#define MCP_16MHz_500kBPS_CFG3 (0x06)

#define MCP_16MHz_250kBPS_CFG1 (0x01)
#define MCP_16MHz_250kBPS_CFG2 (0xb0)
#define MCP_16MHz_250kBPS_CFG3 (0x06)

#define MCP_16MHz_200kBPS_CFG1 (0x00)
#define MCP_16MHz_200kBPS_CFG2 (0xba)
#define MCP_16MHz_200kBPS_CFG3 (0x07)

#define MCP_16MHz_125kBPS_CFG1 (0x01)
#define MCP_16MHz_125kBPS_CFG2 (0xba)
#define MCP_16MHz_125kBPS_CFG3 (0x07)

#define MCP_16MHz_100kBPS_CFG1 (0x03)
#define MCP_16MHz_100kBPS_CFG2 (0xba)
#define MCP_16MHz_100kBPS_CFG3 (0x07)

#define MCP_16MHz_80kBPS_CFG1 (0x03)
#define MCP_16MHz_80kBPS_CFG2 (0xbf)
#define MCP_16MHz_80kBPS_CFG3 (0x07)

#define MCP_16MHz_50kBPS_CFG1 (0x07)
#define MCP_16MHz_50kBPS_CFG2 (0xba)
#define MCP_16MHz_50kBPS_CFG3 (0x07)

#define MCP_16MHz_40kBPS_CFG1 (0x07)
#define MCP_16MHz_40kBPS_CFG2 (0xbf)
#define MCP_16MHz_40kBPS_CFG3 (0x07)

#define MCP_16MHz_20kBPS_CFG1 (0x0f)
#define MCP_16MHz_20kBPS_CFG2 (0xbf)
#define MCP_16MHz_20kBPS_CFG3 (0x07)

#define MCP_16MHz_10kBPS_CFG1 (0x1f)
#define MCP_16MHz_10kBPS_CFG2 (0xbf)
#define MCP_16MHz_10kBPS_CFG3 (0x07)

#define MCP_16MHz_5kBPS_CFG1 (0x3f)
#define MCP_16MHz_5kBPS_CFG2 (0xbf)
#define MCP_16MHz_5kBPS_CFG3 (0x07)



#define MCPDEBUG (0)
#define MCPDEBUG_TXBUF (0)
#define MCP_N_TXBUFFERS (3)

#define MCP_RXBUF_0 (MCP_RXB0SIDH)
#define MCP_RXBUF_1 (MCP_RXB1SIDH)

//#define SPICS 10                                                  <--------------------------------------------------------------------------------------------
//#define MCP2515_SELECT()   digitalWrite(SPICS, LOW)
//#define MCP2515_UNSELECT() digitalWrite(SPICS, HIGH)

#define MCP2515_OK         (0)
#define MCP2515_FAIL       (1)
#define MCP_ALLTXBUSY      (2)

#ifndef CANDEBUG
#define CANDEBUG   0
#endif

#define CANUSELOOP 0

#define CANSENDTIMEOUT (200)                                            /* milliseconds                 */

/*
 *   initial value of gCANAutoProcess
 */
#define CANAUTOPROCESS (1)
#define CANAUTOON  (1)
#define CANAUTOOFF (0)

#define CAN_STDID (0)
#define CAN_EXTID (1)

#define CANDEFAULTIDENT    (0x55CC)
#define CANDEFAULTIDENTEXT (CAN_EXTID)

#define CAN_5KBPS    1
#define CAN_10KBPS   2
#define CAN_20KBPS   3
#define CAN_40KBPS   4
#define CAN_50KBPS   5
#define CAN_80KBPS   6
#define CAN_100KBPS  7
#define CAN_125KBPS  8
#define CAN_200KBPS  9
#define CAN_250KBPS  10
#define CAN_500KBPS  11

#define CAN_OK         (0)
#define CAN_FAILINIT   (1)
#define CAN_FAILTX     (2)
#define CAN_MSGAVAIL   (3)
#define CAN_NOMSG      (4)
#define CAN_CTRLERROR  (5)
#define CAN_GETTXBFTIMEOUT (6)
#define CAN_SENDMSGTIMEOUT (7)
#define CAN_FAIL       (0xff)

#define CAN_MAX_CHAR_IN_MESSAGE (8)

#endif
/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
