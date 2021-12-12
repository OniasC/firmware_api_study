/*
 * robot.h
 *
 *  Created on: Dec 4, 2021
 *      Author: onias
 */

#ifndef APP_ROBOT_H_
#define APP_ROBOT_H_

#include "main.h"
#include "../api/imu/imu.h"
#include "../api/eeprom/eeprom.h"
#include "../api/display_7_seg/display_7seg.h"
#include "../api/surfaceDetection/IRsensor.h"
#include "control/control.h"
#include "mapping/bumpDetection.h"
#include "mapping/edgeDetection.h"

#define ROBOT_NUM_WHEELS 2
#define ROBOT_NUM_SPEEDS 2
#define ROBOT_NUM_IR 3
#define ROBOT_NUM_BUMP 3
#define ROBOT_NUM_STATES 9

typedef struct {
	/*sensors*/
	eeprom_t eeprom;
	imu_t imu;
	float battery;
	edgeSensor_t infraRed[ROBOT_NUM_IR];
	bumpSensor_t bumpSensor[ROBOT_NUM_BUMP];
	display_7seg_t display7Seg;
	nrf24_t radio;

	/*actuators*/
	motor_t wheels[ROBOT_NUM_WHEELS];
	motor_t brushes;

	/*control parameters*/
	float estimatedRobotSpeed[ROBOT_NUM_SPEEDS];
	float commandRobotSpeed[ROBOT_NUM_SPEEDS];
	float speedConvMatrixRobot2Wheels[ROBOT_NUM_SPEEDS][ROBOT_NUM_WHEELS];
	float speedConvMatrixWheels2Robot[ROBOT_NUM_WHEELS][ROBOT_NUM_SPEEDS];
	float estimatedWheelSpeed[ROBOT_NUM_SPEEDS];
	float commandWheelSpeed[ROBOT_NUM_SPEEDS];

	float dynamicState[ROBOT_NUM_STATES]; //dynamicState = {x, y, theta, x_vel, y_vel, theta_vel, x_accel, y_accel, theta_accel}
	float dynamicStateSelfReference[ROBOT_NUM_STATES-3]; //dynamicState = {x_vel, y_vel, theta_vel, x_accel, y_accel, theta_accel}

	control_pid_t pidRobot[ROBOT_NUM_SPEEDS];
	control_pid_t pidWheel[ROBOT_NUM_WHEELS];

} robot_t;

void robot_ctor(robot_t * const robot, 	eeprom_t * const eeprom, imu_t * const imu, motor_t* motorWheels, control_pid_t pidWheels[], control_pid_t pidRobot[], motor_t * const motorBrushes);

void robot_getRobotSpeed(robot_t * const robot);
void robot_setRobotSpeed(robot_t * const robot, float desiredRobotSpeed[]);
void robot_controlRobotSpeed(robot_t * const robot, float desiredRobotSpeed[]);
void robot_controlRobotWheelSpeed(robot_t * const robot);
void robot_convertRobot2Wheel(robot_t * const robot);

void robot_convertWheel2Robot(robot_t * const robot);
#endif /* APP_ROBOT_H_ */
