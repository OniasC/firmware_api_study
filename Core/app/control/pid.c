/*
 * pid.c
 *
 *  Created on: Dec 3, 2021
 *      Author: onias
 */

#include "pid.h"

double PID_calculateOutput(control_pid_t * pid)
{
	double output = 0U;
	double errorP = pid->error[0];
	double errorI = PID_sumIntegralError(pid->error, PID_ERROR_ARRAY);
	double errorD = PID_calcDerivativeError(pid->error, PID_ERROR_ARRAY);
	output = pid->Kd*errorP + pid->Kd*errorI + pid->Kd*errorD;

	return output;
}


double PID_sumIntegralError(int16_t error[], uint8_t arraySize)
{
	double sum;
	for(int i = 0; i < arraySize; i++)
	{
		sum += error[i];
	}
	return sum;
}

double PID_calcDerivativeError(int16_t error[], uint8_t arraySize)
{
	double diff;
	diff = error[0] - error[1];
	return diff;
}

void PID_updateErrorFifo(control_pid_t * const pid)
{
	for(int i = PID_ERROR_ARRAY-1; i > 0; i--)
	{
		pid->error[i] = pid->error[i-1];
	}
	return;
}
