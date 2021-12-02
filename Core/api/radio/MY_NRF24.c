/*
 * MY_NRF24.c
 *
 *  Created on: Sep 14, 2021
 *      Author: onias
 */


/*
Library:					NRF24L01/NRF24L01+
Written by:				Mohamed Yaqoob (MYaqoobEmbedded YouTube Channel)
Date Written:			10/11/2018
Last modified:		18/09/2021 by Onias Castelo
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

//List of header files
#include "MY_NRF24.h"

//*** Variables declaration ***//
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define _BOOL(x) (((x)>0) ? 1:0)

//*** Library variables ***//
static uint64_t pipe0_reading_address;
static bool ack_payload_available; /**< Whether there is an ack payload waiting */

//*** NRF24L01 pins and handles ***//
//Debugging UART handle
//static UART_HandleTypeDef nrf24_huart;

//**** Functions prototypes ****//

void NRF24_ctor (	nrf24_t * const radio, SPI_HandleTypeDef * hspi,
					uint16_t nrf_CE_Pin, GPIO_TypeDef * nrf_CE_GPIO_Port,
					uint16_t nrf_CS_Pin, GPIO_TypeDef * nrf_CS_GPIO_Port,
					uint16_t nrf_int_Pin, GPIO_TypeDef * nrf_int_GPIO_Port,
					nrf24_type_e type)
{
	radio->chip_enable.gpio_pin = nrf_CE_Pin;
	radio->chip_enable.gpio_port = nrf_CE_GPIO_Port;
	radio->chip_select.gpio_pin = nrf_CS_Pin;
	radio->chip_select.gpio_port = nrf_CS_GPIO_Port;
	radio->chip_int.gpio_pin = nrf_int_Pin;
	radio->chip_int.gpio_port = nrf_int_GPIO_Port;
	radio->chip_spi = hspi;
	radio->type = type;

	/*default values*/
	radio->power_level = RF24_PA_m18dB;
	radio->data_rate = RF24_2MBPS;
	radio->crc_length = RF24_CRC_DISABLED;

	radio->payload_size = 32; /**< Fixed size of payloads */
	radio->ack_payload_length = 0;
}

//Microsecond delay function
void NRF24_DelayMicroSeconds(uint32_t uSec)
{
	uint32_t uSecVar = uSec;
	uSecVar = uSecVar* ((SystemCoreClock/1000000)/3);
	while(uSecVar--);
}

//1. Chip Select function
void NRF24_csn(nrf24_t * const radio, int mode)
{
	GPIO_PinState state;
	if(mode) state = GPIO_PIN_SET;
	else state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(radio->chip_select.gpio_port, radio->chip_select.gpio_pin, state);
}
//2. Chip Enable
void NRF24_ce(nrf24_t * const radio, int level)
{
	GPIO_PinState state;
	if(level) state = GPIO_PIN_SET;
	else state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(radio->chip_enable.gpio_port, radio->chip_enable.gpio_pin, state);
}
//3. Read single byte from a register
uint8_t NRF24_read_register(nrf24_t * const radio, uint8_t reg)
{
	uint8_t spiBuf[3];
	uint8_t retData;
	//Put CSN low
	NRF24_csn(radio, 0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;
	HAL_SPI_Transmit(radio->chip_spi, spiBuf, 1, 100);
	//Receive data
	radio->spi_status = HAL_SPI_Receive(radio->chip_spi, &spiBuf[1], 1, 100);
	retData = spiBuf[1];
	//Bring CSN high
	NRF24_csn(radio, 1);
	return retData;
}
//4. Read multiple bytes register
void NRF24_read_registerN(nrf24_t * const radio, uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t spiBuf[3];
	//Put CSN low
	NRF24_csn(radio, 0);
	//Transmit register address
	spiBuf[0] = reg&0x1F;
	//spiStatus = NRF24_SPI_Write(spiBuf, 1);
	radio->spi_status = HAL_SPI_Transmit(radio->chip_spi, spiBuf, 1, 100);
	//Receive data
	radio->spi_status = HAL_SPI_Receive(radio->chip_spi, buf, len, 100);
	//Bring CSN high
	NRF24_csn(radio, 1);
}
//5. Write single byte register
void NRF24_write_register(nrf24_t * const radio, uint8_t reg, uint8_t value)
{
	uint8_t spiBuf[3];
	//Put CSN low
	NRF24_csn(radio, 0);
	//Transmit register address and data
	spiBuf[0] = reg|0x20;
	spiBuf[1] = value;
	radio->spi_status = HAL_SPI_Transmit(radio->chip_spi, spiBuf, 2, 100);
	//Bring CSN high
	NRF24_csn(radio, 1);
}
//6. Write multipl bytes register
void NRF24_write_registerN(nrf24_t * const radio, uint8_t reg, const uint8_t* buf, uint8_t len)
{
	uint8_t spiBuf[3];
	//Put CSN low
	NRF24_csn(radio, 0);
	//Transmit register address and data
	spiBuf[0] = reg|0x20;
	radio->spi_status = HAL_SPI_Transmit(radio->chip_spi, spiBuf, 1, 100);
	radio->spi_status = HAL_SPI_Transmit(radio->chip_spi, (uint8_t*)buf, len, 100);
	//Bring CSN high
	NRF24_csn(radio, 1);
}
//7. Write transmit payload
void NRF24_write_payload(nrf24_t * const radio, const void* buf, uint8_t len)
{
	uint8_t wrPayloadCmd;
	//Bring CSN low
	NRF24_csn(radio, 0);
	//Send Write Tx payload command followed by pbuf data
	wrPayloadCmd = NRF24_CMD_W_TX_PAYLOAD;
	radio->spi_status = HAL_SPI_Transmit(radio->chip_spi, &wrPayloadCmd, 1, 100);
	radio->spi_status = HAL_SPI_Transmit(radio->chip_spi, (uint8_t *)buf, len, 100);
	//Bring CSN high
	NRF24_csn(radio, 1);
}
//8. Read receive payload
void NRF24_read_payload(nrf24_t * const radio, void* buf, uint8_t len)
{
	uint8_t cmdRxBuf;
	//Get data length using payload size
	uint8_t data_len = MIN(len, NRF24_getPayloadSize(radio));
	//Read data from Rx payload buffer
	NRF24_csn(radio, 0);
	cmdRxBuf = NRF24_CMD_R_RX_PAYLOAD;
	HAL_SPI_Transmit(radio->chip_spi, &cmdRxBuf, 1, 100);
	HAL_SPI_Receive(radio->chip_spi, buf, data_len, 100);
	NRF24_csn(radio, 1);
}

//9. Flush Tx buffer
void NRF24_flush_tx(nrf24_t * const radio)
{
	NRF24_write_register(radio, NRF24_CMD_FLUSH_TX, 0xFF);
}
//10. Flush Rx buffer
void NRF24_flush_rx(nrf24_t * const radio)
{
	NRF24_write_register(radio, NRF24_CMD_FLUSH_RX, 0xFF);
}
//11. Get status register value
uint8_t NRF24_get_status(nrf24_t * const radio)
{
	uint8_t statReg;
	statReg = NRF24_read_register(radio, NRF24_REG_STATUS);
	return statReg;
}

//12. Begin function
void NRF24_begin(nrf24_t * const radio)
{
	//Put pins to idle state
	NRF24_csn(radio, 1);
	NRF24_ce(radio, 0);
	//5 ms initial delay
	HAL_Delay(5);

	//**** Soft Reset Registers default values ****//
	NRF24_write_register(radio, NRF24_REG_CONFIG, 		0x08);
	NRF24_write_register(radio, NRF24_REG_EN_AA, 		0x3f);
	NRF24_write_register(radio, NRF24_REG_EN_RXADDR, 	0x03);
	NRF24_write_register(radio, NRF24_REG_SETUP_AW, 	0x03);
	NRF24_write_register(radio, NRF24_REG_SETUP_RETR,	0x03);
	NRF24_write_register(radio, NRF24_REG_RF_CH, 		0x02);
	NRF24_write_register(radio, NRF24_REG_RF_SETUP, 	0x0f);
	NRF24_write_register(radio, NRF24_REG_STATUS, 		0x0e);
	NRF24_write_register(radio, NRF24_REG_OBSERVE_TX, 	0x00);
	NRF24_write_register(radio, NRF24_REG_CD, 			0x00);

	uint8_t pipeAddrVar[5]= {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	NRF24_write_registerN(radio, NRF24_REG_RX_ADDR_P0, pipeAddrVar, 5);

	pipeAddrVar[4] = 0xC2;
	pipeAddrVar[3] = 0xC2;
	pipeAddrVar[2] = 0xC2;
	pipeAddrVar[1] = 0xC2;
	pipeAddrVar[0] = 0xC2;
	NRF24_write_registerN(radio, NRF24_REG_RX_ADDR_P1, 	pipeAddrVar, 5);
	NRF24_write_register(radio, NRF24_REG_RX_ADDR_P2, 	0xC3);
	NRF24_write_register(radio, NRF24_REG_RX_ADDR_P3, 	0xC4);
	NRF24_write_register(radio, NRF24_REG_RX_ADDR_P4, 	0xC5);
	NRF24_write_register(radio, NRF24_REG_RX_ADDR_P5, 	0xC6);

	pipeAddrVar[4] = 0xE7;
	pipeAddrVar[3] = 0xE7;
	pipeAddrVar[2] = 0xE7;
	pipeAddrVar[1] = 0xE7;
	pipeAddrVar[0] = 0xE7;
	NRF24_write_registerN(radio, NRF24_REG_TX_ADDR, pipeAddrVar, 5);
	NRF24_write_register(radio, NRF24_REG_RX_PW_P0, 0);
	NRF24_write_register(radio, NRF24_REG_RX_PW_P1, 0);
	NRF24_write_register(radio, NRF24_REG_RX_PW_P2, 0);
	NRF24_write_register(radio, NRF24_REG_RX_PW_P3, 0);
	NRF24_write_register(radio, NRF24_REG_RX_PW_P4, 0);
	NRF24_write_register(radio, NRF24_REG_RX_PW_P5, 0);

	NRF24_ACTIVATE_cmd(radio);
	NRF24_write_register(radio, 0x1c, 0);
	NRF24_write_register(radio, 0x1d, 0);
	//printRadioSettings(radio);
	//Initialize retries 15 and delay 1250 usec
	NRF24_setRetries(radio, 15, 15);
	//Initialize PA level to max (0dB)
	NRF24_setPALevel(radio, RF24_PA_0dB);
	//Initialize data rate to 1Mbps
	NRF24_setDataRate(radio, RF24_1MBPS);
	//Initialize CRC length to 16-bit (2 bytes)
	NRF24_setCRCLength(radio, RF24_CRC_16);
	//Disable dynamic payload
	NRF24_disableDynamicPayloads(radio);
	//Set payload size
	NRF24_setPayloadSize(radio, 32);

	//Reset status register
	NRF24_resetStatus(radio);
	//Initialize channel to 76
	NRF24_setChannel(radio, 76);
	//Flush buffers
	NRF24_flush_tx(radio);
	NRF24_flush_rx(radio);

	NRF24_powerDown(radio);

}
//13. Listen on open pipes for reading (Must call NRF24_openReadingPipe() first)
void NRF24_startListening(nrf24_t * const radio)
{
	//Power up and set to RX mode
	NRF24_write_register(radio, NRF24_REG_CONFIG, NRF24_read_register(radio, NRF24_REG_CONFIG) | (1UL<<1) |(1UL <<0));
	//Restore pipe 0 address if exists
	if(pipe0_reading_address)
		NRF24_write_registerN(radio, NRF24_REG_RX_ADDR_P0, (uint8_t *)(&pipe0_reading_address), 5);

	//Flush buffers
	NRF24_flush_tx(radio);
	NRF24_flush_rx(radio);
	//Set CE HIGH to start listenning
	NRF24_ce(radio, 1);
	//Wait for 130 uSec for the radio to come on
	NRF24_DelayMicroSeconds(150);
}
//14. Stop listening (essential before any write operation)
void NRF24_stopListening(nrf24_t * const radio)
{
	NRF24_ce(radio, 0);
	NRF24_flush_tx(radio);
	NRF24_flush_rx(radio);
}
//15. Write(Transmit data), returns true if successfully sent
bool NRF24_write(nrf24_t * const radio,  const void* buf, uint8_t len )
{
	bool retStatus;
	//Start writing
	NRF24_resetStatus(radio);
	NRF24_startWrite(radio, buf, len);
	//Data monitor
  uint8_t observe_tx;
  uint8_t status;
  uint32_t sent_at = HAL_GetTick();
	const uint32_t timeout = 10; //ms to wait for timeout
	do
  {
    NRF24_read_registerN(radio, NRF24_REG_OBSERVE_TX,&observe_tx,1);
		//Get status register
		status = NRF24_get_status(radio);
  }
  while( ! ( status & ( _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) ) ) && ( HAL_GetTick() - sent_at < timeout ) );

//	printConfigReg();
//	printStatusReg();

	bool tx_ok, tx_fail;
    NRF24_whatHappened(radio, &tx_ok, &tx_fail, &ack_payload_available);
	retStatus = tx_ok;
	if ( ack_payload_available )
  {
    radio->ack_payload_length = NRF24_getDynamicPayloadSize(radio);
	}

	//Power down
	NRF24_available(radio);
	NRF24_flush_tx(radio);
	return retStatus;
}
//16. Check for available data to read
bool NRF24_available(nrf24_t * const radio)
{
	return NRF24_availablePipe(radio, NULL);
}
//17. Read received data
bool NRF24_read(nrf24_t * const radio,  void* buf, uint8_t len )
{
	NRF24_read_payload(radio,  buf, len );
	uint8_t rxStatus = NRF24_read_register(radio, NRF24_REG_FIFO_STATUS) & _BV(BIT_RX_EMPTY);
	NRF24_flush_rx(radio);
	NRF24_getDynamicPayloadSize(radio);
	return rxStatus;
}
//18. Open Tx pipe for writing (Cannot perform this while Listening, has to call NRF24_stopListening)
void NRF24_openWritingPipe(nrf24_t * const radio, uint64_t address)
{
	NRF24_write_registerN(radio, NRF24_REG_RX_ADDR_P0, (uint8_t *)(&address), 5);
	NRF24_write_registerN(radio, NRF24_REG_TX_ADDR, (uint8_t *)(&address), 5);

	NRF24_write_register(radio, NRF24_REG_RX_PW_P0, MIN(radio->payload_size, NRF24_MAX_PAYLOAD_SIZE));
}
//19. Open reading pipe
void NRF24_openReadingPipe(nrf24_t * const radio, uint8_t number, uint64_t address)
{
	if (number == 0)
    pipe0_reading_address = address;

	if(number <= 6)
	{
		if(number < 2)
		{
			//Address width is 5 bytes
			NRF24_write_registerN(radio, NRF24_ADDR_REGS[number], (uint8_t *)(&address), 5);
		}
		else
		{
			NRF24_write_registerN(radio, NRF24_ADDR_REGS[number], (uint8_t *)(&address), 1);
		}
		//Write payload size
		NRF24_write_register(radio, RF24_RX_PW_PIPE[number], radio->payload_size);
		//Enable pipe
		NRF24_write_register(radio, NRF24_REG_EN_RXADDR, NRF24_read_register(radio, NRF24_REG_EN_RXADDR) | _BV(number));
	}

}
//20 set transmit retries (rf24_Retries_e) and delay
void NRF24_setRetries(nrf24_t * const radio, uint8_t delay, uint8_t count)
{
	NRF24_write_register(radio, NRF24_REG_SETUP_RETR,(delay&0xf)<<BIT_ARD | (count&0xf)<<BIT_ARC);
}

//21. Set RF channel frequency
void NRF24_setChannel(nrf24_t * const radio, uint8_t channel)
{
  NRF24_write_register(radio, NRF24_REG_RF_CH, MIN(channel, NRF24_MAX_CHANNEL));
  radio->channel = channel;
}
//22. Set payload size
void NRF24_setPayloadSize(nrf24_t * const radio, uint8_t size)
{
	radio->payload_size = MIN(size, NRF24_MAX_PAYLOAD_SIZE);
}
//23. Get payload size
uint8_t NRF24_getPayloadSize(nrf24_t * const radio)
{
	return radio->payload_size;
}
//24. Get dynamic payload size, of latest packet received
uint8_t NRF24_getDynamicPayloadSize(nrf24_t * const radio)
{
	return NRF24_read_register(radio, NRF24_CMD_R_RX_PL_WID);
}
//25. Enable payload on Ackknowledge packet
void NRF24_enableAckPayload(nrf24_t * const radio)
{
	//Need to enable dynamic payload and Ack payload together
	 NRF24_write_register(radio, NRF24_REG_FEATURE, NRF24_read_register(radio, NRF24_REG_FEATURE) | _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL) );
	if(!NRF24_read_register(radio, NRF24_REG_FEATURE))
	{
		NRF24_ACTIVATE_cmd(radio);
		NRF24_write_register(radio, NRF24_REG_FEATURE, NRF24_read_register(radio, NRF24_REG_FEATURE) | _BV(BIT_EN_ACK_PAY) | _BV(BIT_EN_DPL) );
	}
	// Enable dynamic payload on pipes 0 & 1
	NRF24_write_register(radio, NRF24_REG_DYNPD, NRF24_read_register(radio, NRF24_REG_DYNPD) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));
}
//26. Enable dynamic payloads
void NRF24_enableDynamicPayloads(nrf24_t * const radio)
{
	//Enable dynamic payload through FEATURE register
	NRF24_write_register(radio, NRF24_REG_FEATURE, NRF24_read_register(radio, NRF24_REG_FEATURE) |  _BV(BIT_EN_DPL) );
	if(!NRF24_read_register(radio, NRF24_REG_FEATURE))
	{
		NRF24_ACTIVATE_cmd(radio);
		NRF24_write_register(radio, NRF24_REG_FEATURE, NRF24_read_register(radio, NRF24_REG_FEATURE) |  _BV(BIT_EN_DPL) );
	}
	//Enable Dynamic payload on all pipes
	NRF24_write_register(radio, NRF24_REG_DYNPD, NRF24_read_register(radio, NRF24_REG_DYNPD) | _BV(BIT_DPL_P5) | _BV(BIT_DPL_P4) | _BV(BIT_DPL_P3) | _BV(BIT_DPL_P2) | _BV(BIT_DPL_P1) | _BV(BIT_DPL_P0));
  radio->dynamic_payloads_enabled = true;
}

void NRF24_disableDynamicPayloads(nrf24_t * const radio)
{
	NRF24_write_register(radio, NRF24_REG_FEATURE, NRF24_read_register(radio, NRF24_REG_FEATURE) &  ~(_BV(BIT_EN_DPL)) );
	//Disable for all pipes
	NRF24_write_register(radio, NRF24_REG_DYNPD, 0);
	radio->dynamic_payloads_enabled = false;
}
//27. Check if module is NRF24L01+ or normal module
bool NRF24_isNRF_Plus(nrf24_t * const radio)
{
	return radio->type;
}
//28. Set Auto Ack for all
void NRF24_setAutoAck(nrf24_t * const radio, bool enable)
{
	if ( enable )
    NRF24_write_register(radio, NRF24_REG_EN_AA, 0x3F);
  else
    NRF24_write_register(radio, NRF24_REG_EN_AA, 0x00);
}
//29. Set Auto Ack for certain pipe
void NRF24_setAutoAckPipe(nrf24_t * const radio,  uint8_t pipe, bool enable )
{
	if ( pipe <= 6 )
  {
    uint8_t en_aa = NRF24_read_register(radio,  NRF24_REG_EN_AA) ;
    if( enable )
    {
      en_aa |= _BV(pipe) ;
    }
    else
    {
      en_aa &= ~_BV(pipe) ;
    }
    NRF24_write_register(radio,  NRF24_REG_EN_AA, en_aa ) ;
  }
}
//30. Set transmit power level
void NRF24_setPALevel(nrf24_t * const radio,  nrf24_pa_dbm_e level )
{
	uint8_t setup = NRF24_read_register(radio, NRF24_REG_RF_SETUP) ;
	setup &= ~(_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;

	// switch uses RAM (evil!)
	if ( level == RF24_PA_0dB)
	{
		setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
	}
	else if ( level == RF24_PA_m6dB )
	{
		setup |= _BV(RF_PWR_HIGH) ;
	}
	else if ( level == RF24_PA_m12dB )
	{
		setup |= _BV(RF_PWR_LOW);
	}
	else if ( level == RF24_PA_m18dB )
	{
	// nothing
	}
	else if ( level == RF24_PA_ERROR )
	{
	// On error, go to maximum PA
		setup |= (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) ;
	}

	NRF24_write_register(radio,  NRF24_REG_RF_SETUP, setup );
	radio->power_level = level;
}
//31. Get transmit power level
nrf24_pa_dbm_e NRF24_getPALevel( nrf24_t * const radio )
{
	nrf24_pa_dbm_e result = RF24_PA_ERROR ;
  uint8_t power = NRF24_read_register(radio, NRF24_REG_RF_SETUP) & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH));

  // switch uses RAM (evil!)
  if ( power == (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH)) )
  {
    result = RF24_PA_0dB ;
  }
  else if ( power == _BV(RF_PWR_HIGH) )
  {
    result = RF24_PA_m6dB ;
  }
  else if ( power == _BV(RF_PWR_LOW) )
  {
    result = RF24_PA_m12dB ;
  }
  else
  {
    result = RF24_PA_m18dB ;
  }

  return result ;
}
//32. Set data rate (250 Kbps, 1Mbps, 2Mbps)
bool NRF24_setDataRate(nrf24_t * const radio, nrf24_datarate_e speed)
{
	bool result = false;
  uint8_t setup = NRF24_read_register(radio, NRF24_REG_RF_SETUP);

  // HIGH and LOW '00' is 1Mbs - our default

  setup &= ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)) ;
  if( speed == RF24_250KBPS )
  {
    // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
    // Making it '10'.

    setup |= _BV( RF_DR_LOW ) ;
  }
  else
  {
    // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
    // Making it '01'
    if ( speed == RF24_2MBPS )
    {

      setup |= _BV(RF_DR_HIGH);
    }
    else
    {
      // 1Mbs

    }
  }

  //setup = setup & 0b10111111; //bit 6 must be 0. maybe this is breaking the nrf?? that is not it..
  NRF24_write_register(radio, NRF24_REG_RF_SETUP, setup);

  // Verify our result
  if ( NRF24_read_register(radio, NRF24_REG_RF_SETUP) == setup )
  {
	radio->data_rate = speed;
    result = true;
  }
  else
  {

  }

  return result;
}
//33. Get data rate
nrf24_datarate_e NRF24_getDataRate( nrf24_t * const radio )
{
	nrf24_datarate_e result ;
  uint8_t dr = NRF24_read_register(radio, NRF24_REG_RF_SETUP) & (_BV(RF_DR_LOW) | _BV(RF_DR_HIGH));

  // switch uses RAM (evil!)
  // Order matters in our case below
  if ( dr == _BV(RF_DR_LOW) )
  {
    // '10' = 250KBPS
    result = RF24_250KBPS ;
  }
  else if ( dr == _BV(RF_DR_HIGH) )
  {
    // '01' = 2MBPS
    result = RF24_2MBPS ;
  }
  else
  {
    // '00' = 1MBPS
    result = RF24_1MBPS ;
  }
  if (result != radio->data_rate) return 0; //ERROR
  return result ;
}
//34. Set crc length (disable, 8-bits or 16-bits)
void NRF24_setCRCLength(nrf24_t * const radio, nrf24_crclength_e length)
{
	uint8_t config = NRF24_read_register(radio, NRF24_REG_CONFIG) & ~( _BV(BIT_CRCO) | _BV(BIT_EN_CRC)) ;

  // switch uses RAM
  if ( length == RF24_CRC_DISABLED )
  {
    // Do nothing, we turned it off above.
  }
  else if ( length == RF24_CRC_8 )
  {
    config |= _BV(BIT_EN_CRC);
  }
  else
  {
    config |= _BV(BIT_EN_CRC);
    config |= _BV( BIT_CRCO );
  }
  NRF24_write_register(radio,  NRF24_REG_CONFIG, config );
}
//35. Get CRC length
nrf24_crclength_e NRF24_getCRCLength(nrf24_t * const radio)
{
	nrf24_crclength_e result = RF24_CRC_DISABLED;
  uint8_t config = NRF24_read_register(radio, NRF24_REG_CONFIG) & ( _BV(BIT_CRCO) | _BV(BIT_EN_CRC)) ;

  if ( config & _BV(BIT_EN_CRC ) )
  {
    if ( config & _BV(BIT_CRCO) )
      result = RF24_CRC_16;
    else
      result = RF24_CRC_8;
  }

  return result;
}
//36. Disable CRC
void NRF24_disableCRC( nrf24_t * const radio )
{
	uint8_t disable = NRF24_read_register(radio, NRF24_REG_CONFIG) & ~_BV(BIT_EN_CRC) ;
  NRF24_write_register(radio,  NRF24_REG_CONFIG, disable ) ;
}
//37. power up
void NRF24_powerUp(nrf24_t * const radio)
{
	NRF24_write_register(radio, NRF24_REG_CONFIG, NRF24_read_register(radio, NRF24_REG_CONFIG) | _BV(BIT_PWR_UP));
}
//38. power down
void NRF24_powerDown(nrf24_t * const radio)
{
	NRF24_write_register(radio, NRF24_REG_CONFIG, NRF24_read_register(radio, NRF24_REG_CONFIG) & ~_BV(BIT_PWR_UP));
}
//39. Check if data are available and on which pipe (Use this for multiple rx pipes)
bool NRF24_availablePipe(nrf24_t * const radio, uint8_t* pipe_num)
{
	uint8_t status = NRF24_get_status(radio);

  bool result = ( status & _BV(BIT_RX_DR) );

  if (result)
  {
    // If the caller wants the pipe number, include that
    if ( pipe_num )
      *pipe_num = ( status >> BIT_RX_P_NO ) & 0x7;

    // Clear the status bit
    NRF24_write_register(radio, NRF24_REG_STATUS,_BV(BIT_RX_DR) );

    // Handle ack payload receipt
    if ( status & _BV(BIT_TX_DS) )
    {
      NRF24_write_register(radio, NRF24_REG_STATUS,_BV(BIT_TX_DS));
    }
  }
  return result;
}
//40. Start write (for IRQ mode)
void NRF24_startWrite(nrf24_t * const radio,  const void* buf, uint8_t len )
{
  // Transmitter power-up
  NRF24_ce(radio, 0);
  NRF24_write_register(radio, NRF24_REG_CONFIG, ( NRF24_read_register(radio, NRF24_REG_CONFIG) | _BV(BIT_PWR_UP) ) & ~_BV(BIT_PRIM_RX) );
  NRF24_ce(radio, 1);
  NRF24_DelayMicroSeconds(150);

  // Send the payload
  NRF24_write_payload(radio,  buf, len );

  // Enable Tx for 15usec
  NRF24_ce(radio, 1);
  NRF24_DelayMicroSeconds(15);
  NRF24_ce(radio, 0);
}
//41. Write acknowledge payload
void NRF24_writeAckPayload(nrf24_t * const radio, uint8_t pipe, const void* buf, uint8_t len)
{
	const uint8_t* current = (uint8_t *)buf;
	const uint8_t max_payload_size = 32;
  uint8_t data_len = MIN(len,max_payload_size);

  NRF24_csn(radio, 0);
	NRF24_write_registerN(radio, NRF24_CMD_W_ACK_PAYLOAD | ( pipe & 0x7 ) , current, data_len);
  NRF24_csn(radio, 1);
}
//42. Check if an Ack payload is available
bool NRF24_isAckPayloadAvailable(nrf24_t * const radio)
{
	bool result = ack_payload_available;
  ack_payload_available = false;
  return result;
}
//43. Check interrupt flags
void NRF24_whatHappened(nrf24_t * const radio, bool *tx_ok, bool *tx_fail, bool *rx_ready)
{
	uint8_t status = NRF24_get_status(radio);
	*tx_ok = 0;
	NRF24_write_register(radio, NRF24_REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );
  // Report to the user what happened
  *tx_ok = status & _BV(BIT_TX_DS);
  *tx_fail = status & _BV(BIT_MAX_RT);
  *rx_ready = status & _BV(BIT_RX_DR);
}
//44. Test if there is a carrier on the previous listenning period (useful to check for intereference)
bool NRF24_testCarrier(nrf24_t * const radio)
{
	return NRF24_read_register(radio, NRF24_REG_CD) & 1;
}
//45. Test if a signal carrier exists (=> -64dB), only for NRF24L01+
bool NRF24_testRPD(nrf24_t * const radio)
{
	return NRF24_read_register(radio, REG_RPD) & 1;
}

//46. Reset Status
void NRF24_resetStatus(nrf24_t * const radio)
{
	NRF24_write_register(radio, NRF24_REG_STATUS,_BV(BIT_RX_DR) | _BV(BIT_TX_DS) | _BV(BIT_MAX_RT) );
}

//47. ACTIVATE cmd
void NRF24_ACTIVATE_cmd(nrf24_t * const radio)
{
	uint8_t cmdRxBuf[2];
	//Read data from Rx payload buffer
	NRF24_csn(radio, 0);
	cmdRxBuf[0] = NRF24_CMD_ACTIVATE;
	cmdRxBuf[1] = 0x73;
	HAL_SPI_Transmit(radio->chip_spi, cmdRxBuf, 2, 100);
	NRF24_csn(radio, 1);
}
//48. Get AckPayload Size
uint8_t NRF24_GetAckPayloadSize(nrf24_t * const radio)
{
	return radio->ack_payload_length;
}

bool NRF24_txStandBy(nrf24_t * const radio, uint32_t timeout, bool startTx)
{

    if (startTx) {
    	NRF24_stopListening(radio);
    	NRF24_ce(radio, 1U);
    }
    uint32_t start = HAL_GetTick();
    uint8_t tx_status = NRF24_read_register(radio, NRF24_REG_FIFO_STATUS);
    while (!(tx_status & _BV(BIT_TX_EMPTY))) {
        if (tx_status /*radio->spi_status*/ & _BV(MASK_MAX_RT)) {
        	/*THIS MIGHT BE WRONG TODO: fix this!!
        	 *
        	 * the reference library compares the spi status return value from the last red_registers call
        	 *    with the _BV(MASK_MAX_RT). This case, compare the SPI_STATUS with 0b10000 ( = 0x10)
        	 *    THE LIBRARY SAYS THAT THE STATUS THAT THE TRANSFER() ANSWERS IS THE DATA REGISTER aka the data actually received
        	 *
        	 * So yeah, it has to be fixed!!!
        	 *
        	 * */

        	NRF24_write_register(radio, NRF24_REG_STATUS, _BV(MASK_MAX_RT));
        	NRF24_ce(radio, 0U); // Set re-transmit
        	NRF24_ce(radio, 1U);
            if (HAL_GetTick() - start >= timeout) {
            	NRF24_ce(radio, 0U);
            	NRF24_flush_tx(radio);
                return 0;
            }
        }
        #if defined(FAILURE_HANDLING) || defined(RF24_LINUX)
        if (HAL_GetTick() - start > (timeout + 95)) {
            errNotify();
            #if defined(FAILURE_HANDLING)
            return 0;
            #endif
        }
        #endif
    }

    NRF24_ce(radio, 0U);  //Set STANDBY-I mode
    return 1;

}

void printRadioSettings(nrf24_t * const radio)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n**********************************************\r\n");
	API_DEBUG_MESSAGE(uartTxBuf);
	//a) Get CRC settings - Config Register
	reg8Val = NRF24_read_register(radio, 0x00);
	if(reg8Val & (1 << 3))
	{
		if(reg8Val & (1 << 2)) sprintf(uartTxBuf, "CRC:\r\n		Enabled, 2 Bytes \r\n");
		else sprintf(uartTxBuf, "CRC:\r\n		Enabled, 1 Byte \r\n");
	}
	else
	{
		sprintf(uartTxBuf, "CRC:\r\n		Disabled \r\n");
	}
	API_DEBUG_MESSAGE(uartTxBuf);
	//b) AutoAck on pipes
	reg8Val = NRF24_read_register(radio, 0x01);
	sprintf(uartTxBuf, "ENAA:\r\n		P0:	%d\r\n		P1:	%d\r\n		P2:	%d\r\n		P3:	%d\r\n		P4:	%d\r\n		P5:	%d\r\n",
	_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	API_DEBUG_MESSAGE(uartTxBuf);
	//c) Enabled Rx addresses
	reg8Val = NRF24_read_register(radio, 0x02);
	sprintf(uartTxBuf, "EN_RXADDR:\r\n		P0:	%d\r\n		P1:	%d\r\n		P2:	%d\r\n		P3:	%d\r\n		P4:	%d\r\n		P5:	%d\r\n",
	_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	API_DEBUG_MESSAGE(uartTxBuf);
	//d) Address width
	reg8Val = NRF24_read_register(radio, 0x03)&0x03;
	reg8Val +=2;
	sprintf(uartTxBuf, "SETUP_AW:\r\n		%d bytes \r\n", reg8Val);
	API_DEBUG_MESSAGE(uartTxBuf);
	//e) RF channel
	reg8Val = NRF24_read_register(radio, 0x05);
	sprintf(uartTxBuf, "RF_CH:\r\n		%d CH \r\n", reg8Val&0x7F);
	API_DEBUG_MESSAGE(uartTxBuf);
	//f) Data rate & RF_PWR
	reg8Val = NRF24_read_register(radio, 0x06);
	if(reg8Val & (1 << 3)) sprintf(uartTxBuf, "Data Rate:\r\n		2Mbps \r\n");
	else sprintf(uartTxBuf, "Data Rate:\r\n		1Mbps \r\n");
	API_DEBUG_MESSAGE(uartTxBuf);
	reg8Val &= (3 << 1);
	reg8Val = (reg8Val>>1);
	if(reg8Val == 0) sprintf(uartTxBuf, "RF_PWR:\r\n		-18dB \r\n");
	else if(reg8Val == 1) sprintf(uartTxBuf, "RF_PWR:\r\n		-12dB \r\n");
	else if(reg8Val == 2) sprintf(uartTxBuf, "RF_PWR:\r\n		-6dB \r\n");
	else if(reg8Val == 3) sprintf(uartTxBuf, "RF_PWR:\r\n		0dB \r\n");
	API_DEBUG_MESSAGE(uartTxBuf);
	//g) RX pipes addresses
	uint8_t pipeAddrs[6];
	NRF24_read_registerN(radio, 0x0A, pipeAddrs, 5);
	sprintf(uartTxBuf, "RX_Pipe0 Addrs:\r\n		%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	API_DEBUG_MESSAGE(uartTxBuf);

	NRF24_read_registerN(radio, 0x0A+1, pipeAddrs, 5);
	sprintf(uartTxBuf, "RX_Pipe1 Addrs:\r\n		%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	API_DEBUG_MESSAGE(uartTxBuf);

	NRF24_read_registerN(radio, 0x0A+2, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe2 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	API_DEBUG_MESSAGE(uartTxBuf);

	NRF24_read_registerN(radio, 0x0A+3, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe3 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	API_DEBUG_MESSAGE(uartTxBuf);

	NRF24_read_registerN(radio, 0x0A+4, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe4 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	API_DEBUG_MESSAGE(uartTxBuf);

	NRF24_read_registerN(radio, 0x0A+5, pipeAddrs, 1);
	sprintf(uartTxBuf, "RX_Pipe5 Addrs:\r\n		xx,xx,xx,xx,%02X  \r\n", pipeAddrs[0]);
	API_DEBUG_MESSAGE(uartTxBuf);

	NRF24_read_registerN(radio, 0x0A+6, pipeAddrs, 5);
	sprintf(uartTxBuf, "TX Addrs:\r\n		%02X,%02X,%02X,%02X,%02X  \r\n", pipeAddrs[4], pipeAddrs[3], pipeAddrs[2],pipeAddrs[1],pipeAddrs[0]);
	API_DEBUG_MESSAGE(uartTxBuf);

	//h) RX PW (Payload Width 0 - 32)
	reg8Val = NRF24_read_register(radio, 0x11);
	sprintf(uartTxBuf, "RX_PW_P0:\r\n		%d bytes \r\n", reg8Val&0x3F);
	API_DEBUG_MESSAGE(uartTxBuf);

	reg8Val = NRF24_read_register(radio, 0x11+1);
	sprintf(uartTxBuf, "RX_PW_P1:\r\n		%d bytes \r\n", reg8Val&0x3F);
	API_DEBUG_MESSAGE(uartTxBuf);

	reg8Val = NRF24_read_register(radio, 0x11+2);
	sprintf(uartTxBuf, "RX_PW_P2:\r\n		%d bytes \r\n", reg8Val&0x3F);
	API_DEBUG_MESSAGE(uartTxBuf);

	reg8Val = NRF24_read_register(radio, 0x11+3);
	sprintf(uartTxBuf, "RX_PW_P3:\r\n		%d bytes \r\n", reg8Val&0x3F);
	API_DEBUG_MESSAGE(uartTxBuf);

	reg8Val = NRF24_read_register(radio, 0x11+4);
	sprintf(uartTxBuf, "RX_PW_P4:\r\n		%d bytes \r\n", reg8Val&0x3F);
	API_DEBUG_MESSAGE(uartTxBuf);

	reg8Val = NRF24_read_register(radio, 0x11+5);
	sprintf(uartTxBuf, "RX_PW_P5:\r\n		%d bytes \r\n", reg8Val&0x3F);
	API_DEBUG_MESSAGE(uartTxBuf);

	//i) Dynamic payload enable for each pipe
	reg8Val = NRF24_read_register(radio, 0x1c);
	sprintf(uartTxBuf, "DYNPD_pipe:\r\n		P0:	%d\r\n		P1:	%d\r\n		P2:	%d\r\n		P3:	%d\r\n		P4:	%d\r\n		P5:	%d\r\n",
	_BOOL(reg8Val&(1<<0)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<2)), _BOOL(reg8Val&(1<<3)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<5)));
	API_DEBUG_MESSAGE(uartTxBuf);

	//j) EN_DPL (is Dynamic payload feature enabled ?)
	reg8Val = NRF24_read_register(radio, 0x1d);
	if(reg8Val&(1<<2)) sprintf(uartTxBuf, "EN_DPL:\r\n		Enabled \r\n");
	else sprintf(uartTxBuf, "EN_DPL:\r\n		Disabled \r\n");
	API_DEBUG_MESSAGE(uartTxBuf);

	//k) EN_ACK_PAY
	if(reg8Val&(1<<1)) sprintf(uartTxBuf, "EN_ACK_PAY:\r\n		Enabled \r\n");
	else sprintf(uartTxBuf, "EN_ACK_PAY:\r\n		Disabled \r\n");
	API_DEBUG_MESSAGE(uartTxBuf);


	sprintf(uartTxBuf, "\r\n**********************************************\r\n");
	API_DEBUG_MESSAGE(uartTxBuf);
}

//2. Print Status
/*void printStatusReg(nrf24_t * const radio)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = NRF24_read_register(radio, 0x07);
	sprintf(uartTxBuf, "STATUS reg:\r\n		RX_DR:		%d\r\n		TX_DS:		%d\r\n		MAX_RT:		%d\r\n		RX_P_NO:	%d\r\n		TX_FULL:	%d\r\n",
	_BOOL(reg8Val&(1<<6)), _BOOL(reg8Val&(1<<5)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(3<<1)), _BOOL(reg8Val&(1<<0)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
}*/
//3. Print Config
/*void printConfigReg(nrf24_t * const radio)
{
	uint8_t reg8Val;
	char uartTxBuf[100];

	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = NRF24_read_register(radio, 0x00);
	sprintf(uartTxBuf, "CONFIG reg:\r\n		PWR_UP:		%d\r\n		PRIM_RX:	%d\r\n",
	_BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<0)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);
}*/

//4. Init Variables
/*void nrf24_DebugUART_Init(UART_HandleTypeDef nrf24Uart)
{
	memcpy(&nrf24_huart, &nrf24Uart, sizeof(nrf24Uart));
}*/
//5. FIFO Status
/*void printFIFOstatus(nrf24_t * const radio)
{
	uint8_t reg8Val;
	char uartTxBuf[100];
	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	reg8Val = NRF24_read_register(radio, 0x17);
	sprintf(uartTxBuf, "FIFO Status reg:\r\n		TX_FULL:		%d\r\n		TX_EMPTY:		%d\r\n		RX_FULL:		%d\r\n		RX_EMPTY:		%d\r\n",
	_BOOL(reg8Val&(1<<5)), _BOOL(reg8Val&(1<<4)), _BOOL(reg8Val&(1<<1)), _BOOL(reg8Val&(1<<0)));
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

	sprintf(uartTxBuf, "\r\n-------------------------\r\n");
	HAL_UART_Transmit(&nrf24_huart, (uint8_t *)uartTxBuf, strlen(uartTxBuf), 10);

}*/
