/*
 * IRsensor.c
 *
 *  Created on: Dec 11, 2021
 *      Author: onias
 */

#include "IRsensor.h"

void irSensor_ctor(irSensor_t * const irSensor, uint16_t pin_Pin, GPIO_TypeDef * pin_Port)
{
	irSensor->pin.gpio_pin = pin_Pin;
	irSensor->pin.gpio_port = pin_Port;
}

