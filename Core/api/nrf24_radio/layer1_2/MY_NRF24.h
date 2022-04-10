/*
 * MY_NRF24.h
 *
 *  Created on: Sep 14, 2021
 *      Author: onias
 */
/*
Library:					NRF24L01/NRF24L01+
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			10/11/2018
Last modified:		-/-
Description:			This is an STM32 device driver library for the NRF24L01 Nordic Radio transceiver, using STM HAL libraries
References:				This library was written based on the Arduino NRF24 Open-Source library by J. Coliz and the NRF24 datasheet
										- https://github.com/maniacbug/RF24
										- https://www.sparkfun.com/datasheets/Components/SMD/nRF24L01Pluss_Preliminary_Product_Specification_v1_0.pdf

* Copyright (C) 2018 - M. Yaqoob
   This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
   of the GNU General Public Licenseversion 3 as published by the Free Software Foundation.

   This software library is shared with puplic for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
   or indirectly by this software, read more about this on the GNU General Public License.
*/

#ifndef SRC_API_MY_NRF24_H_
#define SRC_API_MY_NRF24_H_

//List of header files
#include "main.h"
#include "../../api.h"
#include "../../states.h"
#include "../../Error_Report.h"

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "../../nrf24_radio/layer1_2/nRF24L01.h"

#define NRF24_MAX_PAYLOAD_SIZE	32
#define NRF24_MAX_CHANNEL		127

//**** TypeDefs ****//
//1. Power Amplifier function, NRF24_setPALevel()
typedef enum {
	RF24_PA_m18dB = 0,
	RF24_PA_m12dB,
	RF24_PA_m6dB,
	RF24_PA_0dB,
	RF24_PA_ERROR
} nrf24_pa_dbm_e ;
//2. NRF24_setDataRate() input
typedef enum {
	RF24_1MBPS = 0,
	RF24_2MBPS,
	RF24_250KBPS
} nrf24_datarate_e;
//3. NRF24_setCRCLength() input
typedef enum {
	RF24_CRC_DISABLED = 0,
	RF24_CRC_8,
	RF24_CRC_16
} nrf24_crclength_e;
typedef enum {
	NRF24L01,
	NRF24L01p
} nrf24_type_e;

typedef enum {
	RF24_MODE_RX,
	RF24_MODE_TX
} nrf24_mode_e;

//4. Pipe address registers
static const uint8_t NRF24_ADDR_REGS[7] = {
		NRF24_REG_RX_ADDR_P0,
		NRF24_REG_RX_ADDR_P1,
		NRF24_REG_RX_ADDR_P2,
		NRF24_REG_RX_ADDR_P3,
		NRF24_REG_RX_ADDR_P4,
		NRF24_REG_RX_ADDR_P5,
		NRF24_REG_TX_ADDR
};
//5. RX_PW_Px registers addresses
static const uint8_t RF24_RX_PW_PIPE[6] = {
		NRF24_REG_RX_PW_P0,
		NRF24_REG_RX_PW_P1,
		NRF24_REG_RX_PW_P2,
		NRF24_REG_RX_PW_P3,
		NRF24_REG_RX_PW_P4,
		NRF24_REG_RX_PW_P5
};

/*TODO: fix order of struct to help with packing*/
typedef struct {
	SPI_HandleTypeDef * chip_spi;

	io_pin_t chip_enable;
	io_pin_t chip_select;
	io_pin_t chip_int;
	nrf24_pa_dbm_e power_level;
	nrf24_datarate_e data_rate;
	nrf24_crclength_e crc_length;

	uint8_t channel;
	uint8_t payload_size; /**< Fixed size of payloads */
	uint8_t ack_payload_length;
	nrf24_mode_e mode;
	bool enableAutoAck;
	bool dynamic_payloads_enabled; /**< Whether dynamic payloads are enabled. */
	bool ack_payloads_enabled;
	uint64_t pipe0_reading_address; /**< Last address set on pipe 0 for reading. */
	uint64_t writing_address; /**< Last address set on tx fifo. */
	/*TODO: adapt to use this in the code (will reflect on network layer later on)*/
	uint8_t pipes1to5_MSB_reading_address[4]; /**< Last 4 MSB address set on pipes 1-5 for reading. */
	uint8_t pipes1to5_LSB_reading_address[5]; /**< Last LSB address set on pipes 1-5 for reading. */
	uint8_t addr_width; /**< The address width to use - 3,4 or 5 bytes. */
	uint8_t config_reg; /**< For storing the value of the NRF_CONFIG register */
	//uint32_t spi_speed; /**< SPI Bus Speed */
	nrf24_type_e type;

	HAL_StatusTypeDef spi_status;
	//uint8_t spi_status;
	radio_status_e status;
} nrf24_t;

//**** Functions prototypes ****//

//nrf24 constructor
void NRF24_ctor(nrf24_t * const radio, SPI_HandleTypeDef * hspi,
					uint16_t nrf_CE_Pin, GPIO_TypeDef * nrf_CE_GPIO_Port,
					uint16_t nrf_CS_Pin, GPIO_TypeDef * nrf_CS_GPIO_Port,
					uint16_t nrf_int_Pin, GPIO_TypeDef * nrf_int_GPIO_Port,
					nrf24_type_e type);

//Microsecond delay function
void NRF24_DelayMicroSeconds(uint32_t uSec);

//1. Chip Select function
void NRF24_csn(nrf24_t * const radio, int mode);
//2. Chip Enable
void NRF24_ce(nrf24_t * const radio, int level);
//3. Read single byte from a register
uint8_t NRF24_readRegister(nrf24_t * const radio, uint8_t reg);
uint8_t NRF24_readRegister_v2(nrf24_t * const radio, uint8_t reg, uint8_t * ret);
//4. Read multiple bytes register
void NRF24_readRegisterN(nrf24_t * const radio, uint8_t reg, uint8_t *buf, uint8_t len);
//5. Write single byte register
void NRF24_writeRegister(nrf24_t * const radio, uint8_t reg, uint8_t value);

//6. Write multipl bytes register
void NRF24_writeRegisterN(nrf24_t * const radio, uint8_t reg, const uint8_t* buf, uint8_t len);
//7. Write transmit payload
void NRF24_writePayload(nrf24_t * const radio, const void* buf, uint8_t len);
//8. Read receive payload
void NRF24_readPayload(nrf24_t * const radio, void* buf, uint8_t len);
//9. Flush Tx buffer
void NRF24_flush_tx(nrf24_t * const radio);
//10. Flush Rx buffer
void NRF24_flush_rx(nrf24_t * const radio);
//11. Get status register value
uint8_t NRF24_get_status(nrf24_t * const radio);

//12. Begin function
void NRF24_begin(nrf24_t * const radio);
//13. Listen on open pipes for reading (Must call NRF24_openReadingPipe() first)
void NRF24_startListening(nrf24_t * const radio);
//14. Stop listening (essential before any write operation)
void NRF24_stopListening(nrf24_t * const radio);

//15. Write(Transmit data), returns true if successfully sent
bool NRF24_write(nrf24_t * const radio, const void* buf, uint8_t len );
//16. Check for available data to read
bool NRF24_available(nrf24_t * const radio);
//17. Read received data
bool NRF24_read(nrf24_t * const radio, void* buf, uint8_t len );
//18. Open Tx pipe for writing (Cannot perform this while Listenning, has to call NRF24_stopListening)
void NRF24_openWritingPipe(nrf24_t * const radio, uint64_t address);
//19. Open reading pipe
void NRF24_openReadingPipe(nrf24_t * const radio, uint8_t number, uint64_t address);
//20 set transmit retries (rf24_Retries_e) and delay
void NRF24_setRetries(nrf24_t * const radio, uint8_t delay, uint8_t count);
//21. Set RF channel frequency
void NRF24_setChannel(nrf24_t * const radio, uint8_t channel);
//22. Set payload size
void NRF24_setPayloadSize(nrf24_t * const radio, uint8_t size);
//23. Get payload size
uint8_t NRF24_getPayloadSize(nrf24_t * const radio);
//24. Get dynamic payload size, of latest packet received
uint8_t NRF24_getDynamicPayloadSize(nrf24_t * const radio);
//25. Enable payload on Ackknowledge packet
void NRF24_enableAckPayload(nrf24_t * const radio);
void NRF24_disableAckPayload(nrf24_t * const radio);
//26. Enable dynamic payloads
void NRF24_enableDynamicPayloads(nrf24_t * const radio);
void NRF24_disableDynamicPayloads(nrf24_t * const radio);
//27. Check if module is NRF24L01+ or normal module
bool NRF24_isNRF_Plus(nrf24_t * const radio);
//28. Set Auto Ack for all
void NRF24_setAutoAck(nrf24_t * const radio, bool enable);
//29. Set Auto Ack for certain pipe
void NRF24_setAutoAckPipe(nrf24_t * const radio, uint8_t pipe, bool enable ) ;
//30. Set transmit power level
void NRF24_setPALevel(nrf24_t * const radio, nrf24_pa_dbm_e level ) ;
//31. Get transmit power level
nrf24_pa_dbm_e NRF24_getPALevel(nrf24_t * const radio ) ;
//32. Set data rate (250 Kbps, 1Mbps, 2Mbps)
bool NRF24_setDataRate(nrf24_t * const radio, nrf24_datarate_e speed);
//33. Get data rate
nrf24_datarate_e NRF24_getDataRate(nrf24_t * const radio);
//34. Set crc length (disable, 8-bits or 16-bits)
void NRF24_setCRCLength(nrf24_t * const radio, nrf24_crclength_e length);
//35. Get CRC length
nrf24_crclength_e NRF24_getCRCLength(nrf24_t * const radio);
//36. Disable CRC
void NRF24_disableCRC( nrf24_t * const radio ) ;
//37. power up
void NRF24_powerUp(nrf24_t * const radio) ;
//38. power down
void NRF24_powerDown(nrf24_t * const radio);
//39. Check if data are available and on which pipe (Use this for multiple rx pipes)
bool NRF24_availablePipe(nrf24_t * const radio, uint8_t* pipe_num);
//40. Start write (for IRQ mode)
void NRF24_startWrite(nrf24_t * const radio, const void* buf, uint8_t len );
//41. Write acknowledge payload
void NRF24_writeAckPayload(nrf24_t * const radio, uint8_t pipe, const void* buf, uint8_t len);
//42. Check if an Ack payload is available
bool NRF24_isAckPayloadAvailable(nrf24_t * const radio);
//43. Check interrupt flags
void NRF24_whatHappened(nrf24_t * const radio, bool *tx_ok,bool *tx_fail,bool *rx_ready);
//44. Test if there is a carrier on the previous listenning period (useful to check for intereference)
bool NRF24_testCarrier(nrf24_t * const radio);
//45. Test if a signal carrier exists (=> -64dB), only for NRF24L01+
bool NRF24_testRPD(nrf24_t * const radio) ;
//46. Reset Status
void NRF24_resetStatus(nrf24_t * const radio);
//47. ACTIVATE cmd
void NRF24_ACTIVATE_cmd(nrf24_t * const radio);
//48. Get AckPayload Size
uint8_t NRF24_GetAckPayloadSize(nrf24_t * const radio);

bool NRF24_txStandBy(nrf24_t * const radio, uint32_t timeout, bool startTx);

//**********  DEBUG Functions **********//
//1. Print radio settings
void printRadioSettings(nrf24_t * const radio);
/*//2. Print Status
void printStatusReg(nrf24_t * const radio);
//3. Print Config
void printConfigReg(nrf24_t * const radio);
//4. Init Variables
void nrf24_DebugUART_Init(UART_HandleTypeDef nrf24Uart);
//5. FIFO Status
void printFIFOstatus(nrf24_t * const radio);*/


#endif /* SRC_API_MY_NRF24_H_ */
