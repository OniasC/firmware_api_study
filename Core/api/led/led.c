/*
 * led.c
 *
 *  Created on: Feb 19, 2022
 *      Author: onias
 */

#include "led.h"

led_status_e led_ctorGPIO(led_t * const led, led_mode_e mode, led_polarity_e polarity,
							const uint16_t gpio_pin, GPIO_TypeDef * const gpio_port)
{
	led->gpio.gpio_pin = gpio_pin;
	led->gpio.gpio_port = gpio_port;
	led->polarity = polarity;
	led->mode = LED_MODE_GPIO_RESET;
	led->status = LED_NO_ERROR;

	return led->status;
}

led_status_e led_ctorPWM(led_t * const led, led_mode_e mode,
							TIM_HandleTypeDef *htim, uint32_t channel)

{
	led->pwmPin.Channel = channel;
	led->pwmPin.htim = htim;
	led->mode = LED_MODE_PWM;
	led->status = LED_NO_ERROR;
	return led->status;
}


led_status_e led_set(led_t * const led)
{
	if(led->polarity == LED_POLARITY_DIRECT)
		led->mode = LED_MODE_GPIO_SET;
	else if(led->polarity == LED_POLARITY_INVERSE)
			led->mode = LED_MODE_GPIO_RESET;

	HAL_GPIO_WritePin(led->gpio.gpio_port, led->gpio.gpio_pin, led->mode);
	return led->status;
}

led_status_e led_reset(led_t * const led)
{
	if(led->polarity == LED_POLARITY_DIRECT)
		led->mode = LED_MODE_GPIO_RESET;
	else if(led->polarity == LED_POLARITY_INVERSE)
		led->mode = LED_MODE_GPIO_SET;
	HAL_GPIO_WritePin(led->gpio.gpio_port, led->gpio.gpio_pin, led->mode);
	return led->status;
}

led_status_e led_toggle(led_t * const led)
{
	HAL_GPIO_TogglePin(led->gpio.gpio_port, led->gpio.gpio_pin);
	return led->status;
}

led_status_e led_setPWM(led_t * const led, uint32_t value)
{
	switch (led->pwmPin.Channel)
	{
		case TIM_CHANNEL_1:
			led->pwmPin.htim->Instance->CCR1 = value;
			break;
		case TIM_CHANNEL_2:
			led->pwmPin.htim->Instance->CCR2 = value;
			break;
		case TIM_CHANNEL_3:
			led->pwmPin.htim->Instance->CCR3 = value;
			break;
		case TIM_CHANNEL_4:
			led->pwmPin.htim->Instance->CCR4 = value;
			break;
		default:
			led->status = LED_ERROR;
	}
	return led->status;
}
