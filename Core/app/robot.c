/*
 * robot.c
 *
 *  Created on: Dec 4, 2021
 *      Author: onias
 */

#include "robot.h"

void robot_ctor(robot_t * const robot, 	eeprom_t * const eeprom, imu_t * const imu, motor_t* motor, control_pid_t pidWheel[], control_pid_t pidRobot[], motor_t * const motorBrushes)
{

	//robot->battery = battery;
	robot->eeprom = *eeprom;
	robot->imu = *imu;
	//robot->radio = radio;
	//robot->display7Seg = display7Seg;
	robot->pidRobot[0] = pidRobot[0];
	robot->pidWheel[0] = pidWheel[0];
}

void robot_controlRobotSpeed(robot_t * const robot, float desiredRobotSpeed[])
{
	//float controllerOutput[ROBOT_NUM_SPEEDS];
	robot_getRobotSpeed(robot);

	for (int i = 0; i < ROBOT_NUM_SPEEDS; i++)
	{
		/*PID_updateErrorFifo(&(robot->pidRobot[i]));
		robot->pidRobot[i].error[0] = desiredRobotSpeed[i] - robot->estimatedRobotSpeed[i];
		controllerOutput[i] = PID_calculateOutput(&(robot->pidRobot[i]));
		robot->commandRobotSpeed[i] = controllerOutput[i] + desiredRobotSpeed[i];*/
		robot->commandRobotSpeed[i] = control_PID_SpeedRobot(&(robot->pidRobot[i]), desiredRobotSpeed[i], robot->estimatedRobotSpeed[i]);
	}
	robot_convertRobot2Wheel(robot);
	for (int i = 0; i < ROBOT_NUM_SPEEDS; i++)
	{
		control_PID_SpeedWheel(&(robot->wheels[i]), &(robot->pidWheel[i]), robot->commandWheelSpeed[i]);
	}
}

void robot_controlRobotWheelSpeed(robot_t * const robot)
{
	for (int i = 0; i < ROBOT_NUM_SPEEDS; i++)
	{
		control_PID_SpeedWheel(&(robot->wheels[i]), &(robot->pidWheel[i]), robot->commandWheelSpeed[i]);
	}
}

void robot_setRobotSpeed(robot_t * const robot, float desiredRobotSpeed[])
{
	for (int i = 0; i < ROBOT_NUM_SPEEDS; i++)
	{
		robot->commandRobotSpeed[i] = desiredRobotSpeed[i];
	}
	robot_convertRobot2Wheel(robot);
	for (int i = 0; i < ROBOT_NUM_SPEEDS; i++)
	{
		control_PID_SpeedWheel(&(robot->wheels[i]), &(robot->pidWheel[i]), robot->commandWheelSpeed[i]);
	}
}

void robot_getRobotSpeed(robot_t * const robot)
{
	for (int i=0; i< ROBOT_NUM_WHEELS; i++)
	{
		robot->estimatedRobotSpeed[i] = motor_getSpeed(&(robot->wheels[i]));
	}
	//later more fancy stuff like using a kalman filter and so on
	robot_convertWheel2Robot(robot);
}

void robot_convertRobot2Wheel(robot_t * const robot)
{
	robot->commandWheelSpeed[0] = robot->speedConvMatrixRobot2Wheels[0][0]*robot->commandRobotSpeed[0] +
									robot->speedConvMatrixRobot2Wheels[0][1]*robot->commandRobotSpeed[1];
	robot->commandWheelSpeed[1] = robot->speedConvMatrixRobot2Wheels[1][0]*robot->commandRobotSpeed[0] +
									robot->speedConvMatrixRobot2Wheels[1][1]*robot->commandRobotSpeed[1];
}

void robot_convertWheel2Robot(robot_t * const robot)
{
	robot->estimatedRobotSpeed[0] = robot->speedConvMatrixWheels2Robot[0][0]*robot->estimatedWheelSpeed[0] +
									robot->speedConvMatrixWheels2Robot[0][1]*robot->estimatedWheelSpeed[1];
	robot->estimatedRobotSpeed[1] = robot->speedConvMatrixWheels2Robot[1][0]*robot->estimatedWheelSpeed[0] +
									robot->speedConvMatrixWheels2Robot[1][1]*robot->estimatedWheelSpeed[1];
}
