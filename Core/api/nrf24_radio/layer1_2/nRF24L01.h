/*
 * nRF24L01.h
 *
 *  Created on: Sep 14, 2021
 *      Author: onias
 */
/*
Library:				NRF24L01 software library for STM32 MCUs
Written by:			Mohamed Yaqoob
Date written:		25/10/2018
Last modified:	-/-
Description:
*/

#ifndef SRC_API_NRF24L01_H_
#define SRC_API_NRF24L01_H_

#define _BV(x) (1<<(x))
#define rf24_max(a,b) (a>b?a:b)
#define rf24_min(a,b) (a<b?a:b)

/* Memory Map */
#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_EN_AA       0x01
#define NRF24_REG_EN_RXADDR   0x02
#define NRF24_REG_SETUP_AW    0x03
#define NRF24_REG_SETUP_RETR  0x04
#define NRF24_REG_RF_CH       0x05
#define NRF24_REG_RF_SETUP    0x06
#define NRF24_REG_STATUS      0x07
#define NRF24_REG_OBSERVE_TX  0x08
#define NRF24_REG_CD          0x09
#define NRF24_REG_RX_ADDR_P0  0x0A
#define NRF24_REG_RX_ADDR_P1  0x0B
#define NRF24_REG_RX_ADDR_P2  0x0C
#define NRF24_REG_RX_ADDR_P3  0x0D
#define NRF24_REG_RX_ADDR_P4  0x0E
#define NRF24_REG_RX_ADDR_P5  0x0F
#define NRF24_REG_TX_ADDR     0x10
#define NRF24_REG_RX_PW_P0    0x11
#define NRF24_REG_RX_PW_P1    0x12
#define NRF24_REG_RX_PW_P2    0x13
#define NRF24_REG_RX_PW_P3    0x14
#define NRF24_REG_RX_PW_P4    0x15
#define NRF24_REG_RX_PW_P5    0x16
#define NRF24_REG_FIFO_STATUS 0x17
#define NRF24_REG_DYNPD	    	0x1C
#define NRF24_REG_FEATURE	    0x1D

/* Bit Mnemonics */
#define MASK_RX_DR  6
#define MASK_TX_DS  5
#define MASK_MAX_RT 4
#define BIT_EN_CRC      3
#define BIT_CRCO        2
#define BIT_PWR_UP      1
#define BIT_PRIM_RX     0
#define BIT_ENAA_P5     5
#define BIT_ENAA_P4     4
#define BIT_ENAA_P3     3
#define BIT_ENAA_P2     2
#define BIT_ENAA_P1     1
#define BIT_ENAA_P0     0
#define BIT_ERX_P5      5
#define BIT_ERX_P4      4
#define BIT_ERX_P3      3
#define BIT_ERX_P2      2
#define BIT_ERX_P1      1
#define BIT_ERX_P0      0
#define BIT_AW          0
#define BIT_ARD         4
#define BIT_ARC         0
#define BIT_PLL_LOCK    4
#define BIT_RF_DR       3
#define BIT_RF_PWR      6
#define BIT_RX_DR       6
#define BIT_TX_DS       5
#define BIT_MAX_RT      4
#define BIT_RX_P_NO     1
#define BIT_TX_FULL     0
#define BIT_PLOS_CNT    4
#define BIT_ARC_CNT     0
#define BIT_TX_REUSE    6
#define BIT_FIFO_FULL   5
#define BIT_TX_EMPTY    4
#define BIT_RX_FULL     1
#define BIT_RX_EMPTY    0
#define BIT_DPL_P5	    5
#define BIT_DPL_P4	    4
#define BIT_DPL_P3	    3
#define BIT_DPL_P2	    2
#define BIT_DPL_P1	    1
#define BIT_DPL_P0	    0
#define BIT_EN_DPL	    2
#define BIT_EN_ACK_PAY  1
#define BIT_EN_DYN_ACK  0

/* Instruction Mnemonics */
#define NRF24_CMD_R_REGISTER    0x00
#define NRF24_CMD_W_REGISTER    0x20
#define NRF24_CMD_REGISTER_MASK 0x1F
#define NRF24_CMD_ACTIVATE      0x50
#define NRF24_CMD_R_RX_PL_WID   0x60
#define NRF24_CMD_R_RX_PAYLOAD  0x61
#define NRF24_CMD_W_TX_PAYLOAD  0xA0
#define NRF24_CMD_W_ACK_PAYLOAD 0xA8
#define NRF24_CMD_FLUSH_TX      0xE1
#define NRF24_CMD_FLUSH_RX      0xE2
#define NRF24_CMD_REUSE_TX_PL   0xE3
#define NRF24_CMD_NOP           0xFF

/* Non-P omissions */
#define LNA_HCURR   0

/* P model memory Map */
#define REG_RPD         0x09

/* P model bit Mnemonics */
#define RF_DR_LOW   5
#define RF_DR_HIGH  3
#define RF_PWR_LOW  1
#define RF_PWR_HIGH 2


#endif /* SRC_API_NRF24L01_H_ */
