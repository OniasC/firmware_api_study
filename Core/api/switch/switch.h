/*
 * switch.h
 *
 *  Created on: Apr 18, 2022
 *      Author: onias
 */

#ifndef API_SWITCH_SWITCH_H_
#define API_SWITCH_SWITCH_H_

#include "main.h"
#include "../api.h"
#include "../gpio/gpio.h"

typedef enum{
	SWITCH_POLARITY_LOW,
	SWITCH_POLARITY_HIGH
} polarity_t;

typedef enum{
	SWITCH_MODE_POLLING,
	SWITCH_MODE_IRQ
} switchMode_t;

typedef struct{
	io_pin_t ioPin;
	uint8_t value;
	polarity_t polarity;
	switchMode_t mode;
} switch_t;

void switch_irq_callback(switch_t *const sw);


#endif /* API_SWITCH_SWITCH_H_ */
