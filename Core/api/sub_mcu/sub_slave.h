/*
 * sub_slave.h
 *
 *  Created on: 3 Apr 2022
 *      Author: onias
 */

#ifndef API_SUB_MCU_SUB_SLAVE_H_
#define API_SUB_MCU_SUB_SLAVE_H_


#include "main.h"
#include "../api.h"

#define SLAVE_COMMS_TIMEOUT 100

typedef struct _slave_t {
	SPI_HandleTypeDef *hspi;
	uint8_t 	id;

	float   set_value;
  	float   get_value;

	uint32_t 	slave_select_port;
	uint32_t	slave_select_pin;
	io_pin_t 	chipSelectPin;
	io_pin_t 	irqPin;
} mcuSpiSubsriber_t ;

/* all return 0 if successful */
uint8_t slave_ctor(mcuSpiSubsriber_t* const subs, uint8_t id, SPI_HandleTypeDef *hspi, io_pin_t *const chipSelectPin, io_pin_t*	const irqPin);

uint8_t slave_set_value(mcuSpiSubsriber_t* subs, float v);
uint8_t slave_get_value(mcuSpiSubsriber_t* subs);


#endif /* API_SUB_MCU_SUB_SLAVE_H_ */
