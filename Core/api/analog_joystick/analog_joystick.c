/*
 * analog.c
 *
 *  Created on: Apr 30, 2022
 *      Author: onias
 */

#include "analog_joystick.h"

void analog_joystick_ctor(analog_joystick_t * const joystick, ADC_HandleTypeDef* const hadc, uint32_t* arrayDMA, uint8_t arrayLength)
{
	joystick->hadc1 = hadc;
	joystick->x = &arrayDMA[0];
	joystick->y = &arrayDMA[1];
	joystick->numAxis = arrayLength;
	joystick->array = arrayDMA;
	if(arrayLength == 3U)
	{
		joystick->z = &arrayDMA[2];
	}
	else if (arrayLength > 3U)
	{
		API_ERROR_REPORT(ERR_OTHER, ERR_LVL_ERROR, "not enough parameters in struct");
	}
}

void analog_joystick_calibrationStart(analog_joystick_t * const joystick)
{
	HAL_ADCEx_Calibration_Start(joystick->hadc1);
}

void analog_joystick_start(analog_joystick_t * const joystick)
{
	HAL_ADC_Start_DMA(joystick->hadc1, joystick->array, 1);
}

/*void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//joystick->x = joystick->array[0];

}*/
