/*
 * analog.h
 *
 *  Created on: Apr 30, 2022
 *      Author: onias
 */

#ifndef API_AN_JOYSTICK_ANALOG_JOYSTICK_H_
#define API_AN_JOYSTICK_ANALOG_JOYSTICK_H_

#include "../api.h"
#include "../states.h"
#include "main.h"
#include "../Error_Report.h"

typedef struct {
	ADC_HandleTypeDef* hadc1;
	uint8_t numAxis;
	uint32_t *array; //3 is max!
	uint32_t *x;
	uint32_t *y;
	uint32_t *z;
} analog_joystick_t;



void analog_joystick_ctor(analog_joystick_t * const joystick, ADC_HandleTypeDef* const hadc, uint32_t* arrayDMA, uint8_t arrayLength);

void analog_joystick_calibrationStart(analog_joystick_t * const joystick);

void analog_joystick_start(analog_joystick_t * const joystick);

#endif /* API_AN_JOYSTICK_ANALOG_JOYSTICK_H_ */
