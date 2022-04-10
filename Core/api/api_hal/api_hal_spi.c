/*
 * api_hal_spi.c
 *
 *  Created on: 7 Mar 2022
 *      Author: onias
 */

#include "api_hal_spi.h"

__weak uint8_t spiTransmitReceiveOnce(spi_t * spi, uint8_t reg)
{
	/*
	//Transmit register address
	spiBuf[0] = reg&0x1F;
	HAL_SPI_Transmit(radio->chip_spi, spiBuf, 1, 100);
	//Receive data
	radio->spi_status = HAL_SPI_Receive(radio->chip_spi, &spiBuf[1], 1, 100);
	retData = spiBuf[1];
	return retData;*/
	uint8_t receive = 0U;
	uint32_t timeout = 100U;
	HAL_SPI_TransmitReceive(spi, &reg, &receive, 1, timeout);

	return receive;
}

