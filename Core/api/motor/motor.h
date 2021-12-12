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

typedef enum {
	MOTOR_UNI_DIRECTIONAL,
	MOTOR_BI_DIRECTIONAL
} motor_mode_e;

typedef struct {
	motor_status_e status;
	motor_mode_e mode;
	TIM_HandleTypeDef *encoder;
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

motor_status_e motor_ctor(motor_t * const motor, float max_speed, motor_mode_e mode,
							TIM_HandleTypeDef *htim, uint32_t channel,
							uint16_t dir_io_A_Pin, GPIO_TypeDef * dir_io_A_GPIO_Port,
							uint16_t dir_io_B_Pin, GPIO_TypeDef * dir_io_B_GPIO_Port,
							TIM_HandleTypeDef *htimEncoder);


motor_status_e motor_speed(motor_t * const motor, float speed);

motor_status_e motor_speed_enc(motor_t * const motor, float speed);

float motor_getSpeed(motor_t * const motor);

motor_status_e motor_updateEncoderFifo(motor_t * const motor);
#endif /* API_MOTOR_MOTOR_H_ */
