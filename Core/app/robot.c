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
	robot->eeprom = eeprom;
	robot->imu = imu;
	//robot->radio = radio;
	//robot->display7Seg = display7Seg;
	robot->pidRobot[0] = pidRobot[0];
	robot->pidWheel[0] = pidWheel[0];
}

void robot_getStateEstimates(robot_t * const robot)
{
	float accelCM[3] = {0,0,0}; //a_x, a_y, a_z
	float cosTheta = cos(robot->imuOrientation);
	float sinTheta = sin(robot->imuOrientation);
	/* check how to make this constant to save RAM */
	float imuRotationMatrix[3][3] = {{cosTheta, -sinTheta, 0},
									 {sinTheta, cosTheta,  0},
								     {0,        0,         1}};
	//get imu data
	IMU_readAll(robot->imu);
	//get robot speeds
	robot_getRobotSpeed(robot);
	//get wheel currents ?


	float rotationAccel = ((robot->imu->Gz - robot->dynamicStateLocalReference[2])/robot->controlSampleTimeMs)*1000.0;
	IMU_adjustAccelToCM(robot->imu, imuRotationMatrix, rotationAccel, robot->imuDisplacement, accelCM, 3);

	//wheels data
	robot->measurements[0] = robot->measuredWheelSpeed[0];
	robot->measurements[1] = robot->measuredWheelSpeed[1];

	//imu data
	robot->measurements[2] = accelCM[ROBOT_Y_AXIS]; //y coordinate
	robot->measurements[3] = robot->imu->Gz;


	control_stateEstimatorLinear(robot->kf, robot->kfmEncoder, robot->kfmIMU,
									robot->dynamicStateLocalRefPrevious, ROBOT_NUM_STATES_LOCAL,
									robot->measurements, ROBOT_NUM_STATE_MEASUREMENT,
									robot->commandRobotSpeedPrevious, ROBOT_NUM_SPEEDS,
									robot->dynamicStateLocalReference);

	memcpy(robot->dynamicStateLocalRefPrevious, robot->dynamicStateLocalReference, ROBOT_NUM_STATES_LOCAL);
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

void robot_controlWheelSpeed(robot_t * const robot)
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
		robot->measuredWheelSpeed[i] = motor_getSpeed(&(robot->wheels[i]));
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
	robot->measuredRobotSpeed[0] = robot->speedConvMatrixWheels2Robot[0][0]*robot->measuredWheelSpeed[0] +
									robot->speedConvMatrixWheels2Robot[0][1]*robot->measuredWheelSpeed[1];
	robot->measuredRobotSpeed[1] = robot->speedConvMatrixWheels2Robot[1][0]*robot->measuredWheelSpeed[0] +
									robot->speedConvMatrixWheels2Robot[1][1]*robot->measuredWheelSpeed[1];
}
