/*
 * control.c
 *
 *  Created on: Dec 4, 2021
 *      Author: onias
 */

#include "control.h"


void control_stateEstimatorLinear(kalman_t *kf, kalman_measurement_t *kfmEncoder, kalman_measurement_t *kfmIMU,
							float *previousStates, 	uint8_t numStates,
							float *measurement, 	uint8_t numMeasuments,
							float *inputs, 			uint8_t numInputs,
							float *estimatedStates)
{
	// run kalman filter
	/*
	 * predict stage:
	 * tempState = F * prevState + B * inputs
	 * tempCov	 = F * prevCov * Ft + Q
	 *
	 * update stage:
	 * y          = measurement - H * tempState
	 * S          = H * tempCov *Ht + R
	 * Kgain      = tempCov * Ht * Sinv
	 * newState   = tempState + Kgain * y
	 * newCov     = (I - Kgain * H) * tempCov
	 * */
	matrix_t *x = kalman_get_state_vector(kf);
	matrix_t *u = kalman_get_input_vector(kf);
	matrix_t *zEncoder = kalman_get_measurement_vector(kfmEncoder);
	matrix_t *zIMU = kalman_get_measurement_vector(kfmIMU);

	for(int i = 0; i<numStates; i++)
	{
		matrix_set_ONIAS(x, 0, i, previousStates[i]);
	}
	for(int i =0; i<numInputs; i++)
	{
		matrix_set_ONIAS(u, 0, i, inputs[i]);
	}
	// measure ...
	matrix_set_ONIAS(zEncoder, 0, 0, measurement[0]);
	matrix_set_ONIAS(zEncoder, 0, 1, measurement[1]);

	// measure ...
	matrix_set_ONIAS(zIMU, 0, 0, measurement[2]);
	matrix_set_ONIAS(zIMU, 0, 1, measurement[3]);

	//predict stage
	kalman_predict(kf);
	//update from encoder data
	kalman_correct(kf, kfmEncoder);
	//update from imu data
	kalman_correct(kf, kfmIMU);

	for(int i = 0; i < numStates; i++)
	{
		estimatedStates[i] = x->data[i];
	}
}

float control_PID_SpeedRobot(control_pid_t * const pid, float desiredSpeed, float currentSpeed)
{
	float controllerOutput;
	PID_updateErrorFifo(pid);
	pid->error[0] = desiredSpeed - currentSpeed;
	controllerOutput = PID_calculateOutput(pid);
	return (controllerOutput + desiredSpeed);
}

void control_PID_SpeedWheel(motor_t * const motor, control_pid_t * const pid, double desiredSpeed)
{
	double currentWheelSpeed = (double)motor->deltaEncTicks/MOTOR_ENCODER_STORAGE;
	double motorInput;
	PID_updateErrorFifo(pid);
	pid->error[0] = currentWheelSpeed - desiredSpeed*SI_TO_TICKS; //either this or the inverse of the error being in units of m/s and not ticks/ms
	motorInput = PID_calculateOutput(pid);
	motor_setSpeedClosedLoop(motor, (float)motorInput);
}

