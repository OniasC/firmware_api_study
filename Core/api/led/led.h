/*
 * led.h
 *
 *  Created on: Feb 19, 2022
 *      Author: onias
 */

#ifndef API_LED_LED_H_
#define API_LED_LED_H_

#include "main.h"
#include "../api.h"
#include "../states.h"

typedef enum {
	LED_POLARITY_DIRECT = 0U,
	LED_POLARITY_INVERSE = 1U
} led_polarity_e;


typedef enum {
	LED_MODE_GPIO_RESET = 0U,
	LED_MODE_GPIO_SET = 1U,
	LED_MODE_PWM = 2U,
} led_mode_e;

typedef struct {
	led_status_e status;
	led_mode_e mode;
	led_polarity_e polarity;
	pwm_t pwmPin;
	io_pin_t gpio;

	/*
	 * more stuff about pwm modes in pwm.
	 *
	 * */
	uint16_t refresh_freq;
} led_t;

led_status_e led_ctorGPIO(led_t * const led, led_mode_e mode,
							led_polarity_e polarity,
							const uint16_t gpio_pin, GPIO_TypeDef * const gpio_port);

led_status_e led_ctorPWM(led_t * const led, led_mode_e mode,
							TIM_HandleTypeDef *htim, uint32_t channel);

led_status_e led_set(led_t * const led);

led_status_e led_reset(led_t * const led);

led_status_e led_setPWM(led_t * const led, uint32_t value);

#endif /* API_LED_LED_H_ */
