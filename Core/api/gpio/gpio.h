/*
 * gpio.h
 *
 *  Created on: Apr 26, 2022
 *      Author: onias
 */

#ifndef API_GPIO_GPIO_H_
#define API_GPIO_GPIO_H_

#include "main.h"
#include "../api.h"

void gpio_write(io_pin_t * const gpio, uint8_t value);

void gpio_set(io_pin_t * const gpio);

void gpio_reset(io_pin_t * const gpio);

void gpio_toggle(io_pin_t * const gpio);

uint8_t gpio_read(io_pin_t * const gpio);

#endif /* API_GPIO_GPIO_H_ */
