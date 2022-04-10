/*
 * api_hal_spi.h
 *
 *  Created on: 7 Mar 2022
 *      Author: onias
 */

#ifndef API_API_HAL_API_HAL_SPI_H_
#define API_API_HAL_API_HAL_SPI_H_

#include "../api.h"

#define spi_t SPI_HandleTypeDef

__weak uint8_t spiTransmitReceiveOnce(spi_t * spi, uint8_t reg);



#endif /* API_API_HAL_API_HAL_SPI_H_ */
