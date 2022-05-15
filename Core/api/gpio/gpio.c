/*
 * gpio.c
 *
 *  Created on: Apr 26, 2022
 *      Author: onias
 */

#include "gpio.h"

void gpio_write(io_pin_t * const gpio, uint8_t value)
{
	if((value == 0U) || (value == 1U))
	{
		HAL_GPIO_WritePin(gpio->gpio_port, gpio->gpio_pin, value);
	}
}

void gpio_set(io_pin_t * const gpio)
{
	HAL_GPIO_WritePin(gpio->gpio_port, gpio->gpio_pin, 1U);
}

void gpio_reset(io_pin_t * const gpio)
{
	HAL_GPIO_WritePin(gpio->gpio_port, gpio->gpio_pin, 0U);
}

void gpio_toggle(io_pin_t * const gpio)
{
	HAL_GPIO_TogglePin(gpio->gpio_port, gpio->gpio_pin);
}

uint8_t gpio_read(io_pin_t * const gpio)
{
	return HAL_GPIO_ReadPin(gpio->gpio_port, gpio->gpio_pin);
}
