/*
 * motor.c
 *
 *  Created on: Oct 9, 2021
 *      Author: onias
 */

#include "motor.h"


motor_status_e motor_ctor(motor_t * const motor, float max_speed, motor_mode_e mode,
							TIM_HandleTypeDef *htimPWM, uint32_t channel,
							uint16_t dir_io_A_Pin, GPIO_TypeDef * dir_io_A_GPIO_Port,
							uint16_t dir_io_B_Pin, GPIO_TypeDef * dir_io_B_GPIO_Port,
							TIM_HandleTypeDef *htimEncoder)
{
	motor->mode = mode;
	motor->power.htim = htimPWM;
	motor->power.Channel = channel;
	if (motor->mode == MOTOR_BI_DIRECTIONAL)
	{
		motor->directionA.gpio_pin = dir_io_A_Pin;
		motor->directionA.gpio_port = dir_io_A_GPIO_Port;
		motor->directionB.gpio_pin = dir_io_B_Pin;
		motor->directionB.gpio_port = dir_io_B_GPIO_Port;
	}
	motor->max_speed = max_speed;
	motor->speed = 0.0;
	motor->current = 0.0;
	motor->encoder = htimEncoder;
	for(int i = 0; i < (MOTOR_ENCODER_STORAGE-1); i++)
	{
		motor->encCounter[i] = 0;
	}
	motor->deltaEncTicks = 0;
	motor->status = MOTOR_NO_ERROR;
	return motor->status;
}

motor_status_e motor_speed(motor_t * const motor, float speed)
{
	float fraction;
	uint32_t volume;
	if (speed > motor->max_speed) speed = motor->max_speed;
	if (speed < 0 ) speed = 0.0;
	fraction = speed / motor->max_speed;
	volume = motor->power.htim->Init.Period * fraction;
	if (motor->power.Channel == TIM_CHANNEL_1)
		motor->power.htim->Instance->CCR1 = volume; // vary the duty cycle
	else if (motor->power.Channel == TIM_CHANNEL_2)
		motor->power.htim->Instance->CCR2 = volume; // vary the duty cycle
	else if (motor->power.Channel == TIM_CHANNEL_3)
		motor->power.htim->Instance->CCR3 = volume; // vary the duty cycle
	else if (motor->power.Channel == TIM_CHANNEL_4)
		motor->power.htim->Instance->CCR4 = volume; // vary the duty cycle
	return motor->status;
}

motor_status_e motor_speed_enc(motor_t * const motor, float speed)
{
	float fraction;
	uint32_t volume;
	if (speed > 0)
	{
		HAL_GPIO_WritePin(motor->directionA.gpio_port, motor->directionA.gpio_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->directionB.gpio_port, motor->directionB.gpio_pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(motor->directionB.gpio_port, motor->directionB.gpio_pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(motor->directionA.gpio_port, motor->directionA.gpio_pin, GPIO_PIN_RESET);
		speed = - speed;
	}
	if (speed > motor->max_speed) speed = motor->max_speed;
	if (speed < - motor->max_speed ) speed = -motor->max_speed;
	fraction = speed / motor->max_speed;
	volume = motor->power.htim->Init.Period * fraction;
	if (motor->power.Channel == TIM_CHANNEL_1)
		motor->power.htim->Instance->CCR1 = volume; // vary the duty cycle
	else if (motor->power.Channel == TIM_CHANNEL_2)
		motor->power.htim->Instance->CCR2 = volume; // vary the duty cycle
	else if (motor->power.Channel == TIM_CHANNEL_3)
		motor->power.htim->Instance->CCR3 = volume; // vary the duty cycle
	else if (motor->power.Channel == TIM_CHANNEL_4)
		motor->power.htim->Instance->CCR4 = volume; // vary the duty cycle
	return motor->status;
}

float motor_getSpeed(motor_t * const motor)
{
	motor->speedSI = ((float)motor->deltaEncTicks/MOTOR_ENCODER_STORAGE)*SI_TO_TICKS;
	return motor->speedSI;
}

motor_status_e motor_updateEncoderFifo(motor_t * const motor)
{
	for(int i = MOTOR_ENCODER_STORAGE-1; i > 0; i--)
	{
		motor->encCounter[i] = motor->encCounter[i-1];
	}
	return motor->status;
}
