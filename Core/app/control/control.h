/*
 * control.h
 *
 *  Created on: Dec 4, 2021
 *      Author: onias
 */

#ifndef APP_CONTROL_CONTROL_H_
#define APP_CONTROL_CONTROL_H_


#include "../api/motor/motor.h"
#include "../app/control/kalman_robot.h"
#include "pid.h"


void control_stateEstimatorLinear(kalman_t *kf,
								kalman_measurement_t *kfmEncoder, kalman_measurement_t *kfmIMU,
		                        float *previousStates, 	uint8_t numStates,
								float *measurement, 	uint8_t numMeasuments,
								float *inputs, 			uint8_t numInputs,
								float *estimatedStates);

float control_PID_SpeedRobot(control_pid_t * const pid, float desiredSpeed, float currentSpeed);

void control_PID_SpeedWheel(motor_t * const motor, control_pid_t * const pid, double desiredSpeed);

#endif /* APP_CONTROL_CONTROL_H_ */
