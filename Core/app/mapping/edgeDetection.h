/*
 * edgeDetection.h
 *
 *  Created on: Dec 4, 2021
 *      Author: onias
 */

#ifndef APP_MAPPING_EDGEDETECTION_H_
#define APP_MAPPING_EDGEDETECTION_H_

#include "../../api/surfaceDetection/IRsensor.h"

typedef enum{
	NO_EDGE = 0U,
	EDGE_FRONT = 1<<0,
	EDGE_RIGHT = 1<<1,
	EDGE_LEFT = 1<<2
} edgeSensor_e;

typedef struct {
	irSensor_t irSensor;
} edgeSensor_t;

edgeSensor_e detectEdge(edgeSensor_t edgeArray[], uint8_t sizeArray);

#endif /* APP_MAPPING_EDGEDETECTION_H_ */
