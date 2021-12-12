/*
 * bumpDetection.h
 *
 *  Created on: Dec 4, 2021
 *      Author: onias
 */

#ifndef APP_MAPPING_BUMPDETECTION_H_
#define APP_MAPPING_BUMPDETECTION_H_

#include"../../api/surfaceDetection/IRsensor.h"

typedef enum{
	NO_BUMP = 0U,
	BUMP_FRONT = 1<<0,
	BUMP_RIGHT = 1<<1,
	BUMP_LEFT = 1<<2
} bumpSensor_e;

typedef struct {
	irSensor_t irSensor;
} bumpSensor_t;

bumpSensor_e detectObstacle(bumpSensor_t bumpArray[], uint8_t sizeArray);

#endif /* APP_MAPPING_BUMPDETECTION_H_ */
