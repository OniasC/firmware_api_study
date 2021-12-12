/*
 * IRsensor.h
 *
 *  Created on: Dec 11, 2021
 *      Author: onias
 */

#ifndef API_SURFACEDETECTION_IRSENSOR_H_
#define API_SURFACEDETECTION_IRSENSOR_H_

#include "../api.h"

typedef struct {
	io_pin_t pin;
	uint8_t sensorOutput;
} irSensor_t;

void irSensor_ctor(irSensor_t * const irSensor, uint16_t pin_Pin, GPIO_TypeDef * pin_Port);

#endif /* API_SURFACEDETECTION_IRSENSOR_H_ */
