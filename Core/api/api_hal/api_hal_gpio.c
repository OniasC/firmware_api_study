/*
 * api_hal_gpio.c
 *
 *  Created on: 7 Mar 2022
 *      Author: onias
 */

#include "api_hal_gpio.h"

__weak void digitalWrite(io_pin_t pin, uint8_t mode)
{
	HAL_GPIO_WritePin(pin.gpio_port, pin.gpio_pin, mode);
}

