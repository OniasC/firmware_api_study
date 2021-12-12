/*
 * pid.h
 *
 *  Created on: Dec 3, 2021
 *      Author: onias
 */

#ifndef APP_CONTROL_PID_H_
#define APP_CONTROL_PID_H_

#define PID_ERROR_ARRAY 15

#include "main.h"

typedef struct{
	uint16_t Kp;
	uint16_t Ki;
	uint16_t Kd;
	int16_t error[PID_ERROR_ARRAY];
} control_pid_t;

double PID_calculateOutput(control_pid_t * const pid);

double PID_sumIntegralError(int16_t error[], uint8_t arraySize);

double PID_calcDerivativeError(int16_t error[], uint8_t arraySize);

void PID_updateErrorFifo(control_pid_t * const pid);

#endif /* APP_CONTROL_PID_H_ */
