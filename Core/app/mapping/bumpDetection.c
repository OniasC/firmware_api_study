/*
 * bumpDetection.c
 *
 *  Created on: Dec 4, 2021
 *      Author: onias
 */

#include "bumpDetection.h"


bumpSensor_e detectObstacle(bumpSensor_t bumpArray[], uint8_t sizeArray)
{
	bumpSensor_e returnVal = 0U;
	for (int i=0; i<sizeArray; i++)
	{
		if(HAL_GPIO_ReadPin(bumpArray[i].irSensor.pin.gpio_port, bumpArray[i].irSensor.pin.gpio_pin) == GPIO_PIN_SET)
		{
			returnVal |= 1<<i;
		}
	}
	return returnVal;
}
