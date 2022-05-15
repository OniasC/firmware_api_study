/*
 * buzzer.h
 *
 *  Created on: Sep 18, 2021
 *      Author: onias
 */

#ifndef API_BUZZER_BUZZER_H_
#define API_BUZZER_BUZZER_H_

#include "main.h"
#include "../api.h"
#include "../states.h"
#include "themes.h"

typedef struct {
	pwm_t timer_config;
	uint32_t frequency;
	uint8_t volume;
} buzzer_t;

buzzer_status_e buzzer_ctor(buzzer_t * const buzzer, TIM_HandleTypeDef *htim, uint32_t Channel);

void BUZZER_play(buzzer_t * const buzzer);

void BUZZER_SetVolume(buzzer_t * const buzzer, uint32_t volume);

void BUZZER_tone(buzzer_t * const buzzer, uint32_t frequency, uint32_t duration);

void BUZZER_Play_Pirates(buzzer_t * const buzzer);

#endif /* API_BUZZER_BUZZER_H_ */
