/*
 * motor.h
 *
 *  Created on: Oct 9, 2021
 *      Author: onias
 */

#ifndef API_MOTOR_MOTOR_H_
#define API_MOTOR_MOTOR_H_

#include "main.h"
#include "../api.h"
#include "../states.h"

#define MOTOR_ENCODER_STORAGE 13
#define SI_TO_TICKS 3000 //for example

typedef TIM_HandleTypeDef encoder_t;

typedef enum {
	MOTOR_MODE_UNI_DIR_OPEN_LOOP,
	MOTOR_MODE_UNI_DIR_CLOSED_LOOP, //NOT IMPLEMENTED
	MOTOR_MODE_BI_DIR_DRIVECOAST_OPEN_LOOP, //USES TWO PINS: 2 IO (Dir) 1 PWM (Speed)
	MOTOR_MODE_BI_DIR_DRIVECOAST_CLOSED_LOOP,
	MOTOR_MODE_BI_DIR_SIGNMAG_OPEN_LOOP, //USES TWO PINS: 1 IO (Dir) 1 PWM (Speed)
	MOTOR_MODE_BI_DIR_SIGNMAG_CLOSED_LOOP,
} motor_mode_e;

typedef struct {
	motor_status_e status;
	motor_mode_e mode;
	encoder_t *encoder;
	pwm_t power;
	io_pin_t directionA;
	io_pin_t directionB;
	float max_speed;
	uint32_t encCounter[MOTOR_ENCODER_STORAGE];
	float current;
	float speed;
	float speedSI;
	int deltaEncTicks;
} motor_t;

motor_status_e motor_ctorSimple(motor_t * const motor, float max_speed, motor_mode_e mode,
							TIM_HandleTypeDef *htimPWM, uint32_t channel);

motor_status_e motor_ctorDriveCoast(motor_t * const motor, float max_speed, motor_mode_e mode,
									pwm_t * const pwmPin,
									io_pin_t * const dir_io_A,
									io_pin_t * const dir_io_B,
									encoder_t * const htimEncoder);

motor_status_e motor_ctorSignMag(motor_t * const motor, float max_speed, motor_mode_e mode,
								pwm_t * const pwmPin,
								io_pin_t * const dir_io_A,
								encoder_t * const htimEncoder);

motor_status_e motor_setSpeedOpenLoop(motor_t * const motor, float speed);

motor_status_e motor_setSpeedClosedLoop(motor_t * const motor, float speed);

float motor_getSpeed(motor_t * const motor);

motor_status_e motor_updateEncoderFifo(motor_t * const motor);
#endif /* API_MOTOR_MOTOR_H_ */
