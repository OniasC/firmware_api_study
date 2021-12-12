/*
 * edgeDetection.c
 *
 *  Created on: Dec 4, 2021
 *      Author: onias
 */

#include "edgeDetection.h"

edgeSensor_e detectEdge(edgeSensor_t edgeArray[], uint8_t sizeArray)
{
	edgeSensor_e returnVal = 0U;
	for (int i=0; i<sizeArray; i++)
	{
		if(HAL_GPIO_ReadPin(edgeArray[i].irSensor.pin.gpio_port, edgeArray[i].irSensor.pin.gpio_pin) == GPIO_PIN_SET)
		{
			returnVal |= 1<<i;
		}
	}
	return returnVal;
}
