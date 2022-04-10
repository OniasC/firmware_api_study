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
#include "mapping/floorDetection.h"
#include "mapping/wallDetection.h"
#include "../api/nrf24_radio/layer1_2/MY_NRF24.h"

#define ROBOT_NUM_WHEELS 2
#define ROBOT_NUM_SPEEDS 2
#define ROBOT_NUM_IR 3
#define ROBOT_NUM_BUMP 3
#define ROBOT_NUM_STATES_WORLD 9
#define ROBOT_NUM_STATES_LOCAL 4
#define ROBOT_NUM_STATE_MEASUREMENT 4

#define ROBOT_X_AXIS 0
#define ROBOT_Y_AXIS 1
#define ROBOT_Z_AXIS 2


/*
 *         y_axis
 *           ^
 *           |
 *      ____________
 *     /            \
 *     |            |  ->  x_axis
 *     |____________|
 *
 *
 *
 * */
typedef enum {
	robot_status_not_init,
	robot_status_no_error,
	robot_status_error
} robot_status_e;

typedef struct {
	/*sensors*/
	eeprom_t *eeprom;
	imu_t *imu;
	float imuDisplacement[3];
	float imuOrientation;
	float battery;
	edgeSensor_t infraRed[ROBOT_NUM_IR];
	bumpSensor_t bumpSensor[ROBOT_NUM_BUMP];
	display_7seg_t *display7Seg;
	nrf24_t *radio;

	/*actuators*/
	motor_t wheels[ROBOT_NUM_WHEELS];
	motor_t brushes;
	motor_t vaccum;

	/*control parameters*/
	float estimatedRobotSpeed[ROBOT_NUM_SPEEDS];
	float commandRobotSpeed[ROBOT_NUM_SPEEDS];
	float commandRobotSpeedPrevious[ROBOT_NUM_SPEEDS];
	float speedConvMatrixRobot2Wheels[ROBOT_NUM_SPEEDS][ROBOT_NUM_WHEELS];
	float speedConvMatrixWheels2Robot[ROBOT_NUM_WHEELS][ROBOT_NUM_SPEEDS];

	float measuredRobotSpeed[ROBOT_NUM_SPEEDS];
	float measuredWheelSpeed[ROBOT_NUM_SPEEDS];
	float commandWheelSpeed[ROBOT_NUM_SPEEDS];
	uint8_t controlSampleTimeMs;

	float measurements[ROBOT_NUM_STATE_MEASUREMENT]; // measurements = {v_y, w_encoder, a_y, w_imu} //w can come from imu and encoder...
	float dynamicStatePrevious[ROBOT_NUM_STATES_WORLD];
	float dynamicState[ROBOT_NUM_STATES_WORLD]; //dynamicState = {x, y, theta, x_vel, y_vel, theta_vel, x_accel, y_accel, theta_accel}
	float dynamicStateLocalRefPrevious[ROBOT_NUM_STATES_LOCAL]; //dynamicState = {y_vel, theta_vel, y_accel, theta_accel}
	float dynamicStateLocalReference[ROBOT_NUM_STATES_LOCAL]; //dynamicState = {y_vel, theta_vel, y_accel, theta_accel}

	kalman_t *kf;
	kalman_measurement_t *kfmEncoder;
	kalman_measurement_t *kfmIMU;

	control_pid_t pidRobot[ROBOT_NUM_SPEEDS];
	control_pid_t pidWheel[ROBOT_NUM_WHEELS];
	robot_status_e status;
} robot_t;

void robot_ctor(robot_t * const robot, 	eeprom_t * const eeprom, imu_t * const imu, motor_t* motorWheels, control_pid_t pidWheels[], control_pid_t pidRobot[], motor_t * const motorBrushes);

void robot_getStateEstimates(robot_t * const robot);

void robot_getRobotSpeed(robot_t * const robot);
void robot_setRobotSpeed(robot_t * const robot, float desiredRobotSpeed[]);
void robot_controlRobotSpeed(robot_t * const robot, float desiredRobotSpeed[]);
void robot_controlWheelSpeed(robot_t * const robot);

void robot_convertRobot2Wheel(robot_t * const robot);
void robot_convertWheel2Robot(robot_t * const robot);
#endif /* APP_ROBOT_H_ */
