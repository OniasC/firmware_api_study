/*
 * control.c
 *
 *  Created on: Dec 4, 2021
 *      Author: onias
 */

#include "control.h"

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
	motor_speed_enc(motor, (float)motorInput);
}
