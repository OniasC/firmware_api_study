/*
 * bsp.c
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#include "bsp.h"

eeprom_at24c eeprom;
eeprom_t eeprom2;
imu_mpu6050_t imu1;
display_7seg_t display_7seg_io;
display_7seg_cc_t display_7seg_cc_io;

extern I2C_HandleTypeDef hi2c1;

eeprom_status BSP_EEPROM_Init(void)
{
	eeprom.super.status = EEPROM_NOT_INIT;
	eeprom_at24c_ctor(&eeprom,  EEPROM_AT24C02C, &hi2c1, 0);
	eeprom_ctor(&eeprom2, EEPROM_AT24C02C, &hi2c1, 1);
	return eeprom.super.status;
}

imu_status BSP_IMU_Init(void)
{
	return imu_mpu6050_ctor(&imu1, &hi2c1);
}

display_7seg_status BSP_DISPLAY_7SEG_Init(void)
{
	/*#define LED_STATUS_Pin GPIO_PIN_13
	#define LED_STATUS_GPIO_Port GPIOC
	#define DISP_7S_7_Pin GPIO_PIN_12
	#define DISP_7S_7_GPIO_Port GPIOB
	#define DISP_7S_6_Pin GPIO_PIN_13
	#define DISP_7S_6_GPIO_Port GPIOB
	#define DISP_7S_5_Pin GPIO_PIN_14
	#define DISP_7S_5_GPIO_Port GPIOB
	#define DISP_7S_4_Pin GPIO_PIN_15
	#define DISP_7S_4_GPIO_Port GPIOB
	#define DISP_7S_3_Pin GPIO_PIN_8
	#define DISP_7S_3_GPIO_Port GPIOA
	#define DISP_7S_2_Pin GPIO_PIN_9
	#define DISP_7S_2_GPIO_Port GPIOA
	#define DISP_7S_1_Pin GPIO_PIN_10
	#define DISP_7S_1_GPIO_Port GPIOA*/
	display_7seg_cc_io.super.status = DISPLAY_7SEG_IO;

	/*display_7seg_cc_io.middle_horizontal_pin.gpio_pin = DISP_7S_1_Pin;
	display_7seg_cc_io.middle_horizontal_pin.gpio_port = DISP_7S_1_GPIO_Port;
	display_7seg_cc_io.top_left_vertical_pin.gpio_pin = DISP_7S_2_Pin;
	display_7seg_cc_io.top_left_vertical_pin.gpio_port = DISP_7S_2_GPIO_Port;
	display_7seg_cc_io.top_horizontal_pin.gpio_pin = DISP_7S_3_Pin;
	display_7seg_cc_io.top_horizontal_pin.gpio_port = DISP_7S_3_GPIO_Port;
	display_7seg_cc_io.top_right_vertical_pin.gpio_pin = DISP_7S_4_Pin;
	display_7seg_cc_io.top_right_vertical_pin.gpio_port = DISP_7S_4_GPIO_Port;
	display_7seg_cc_io.bottom_left_vertical_pin.gpio_pin = DISP_7S_5_Pin;
	display_7seg_cc_io.bottom_left_vertical_pin.gpio_port = DISP_7S_5_GPIO_Port;
	display_7seg_cc_io.bottom_horizontal_pin.gpio_pin = DISP_7S_6_Pin;
	display_7seg_cc_io.bottom_horizontal_pin.gpio_port = DISP_7S_6_GPIO_Port;
	display_7seg_cc_io.bottom_right_vertical_pin.gpio_pin = DISP_7S_7_Pin;
	display_7seg_cc_io.bottom_right_vertical_pin.gpio_port = DISP_7S_7_GPIO_Port;
	display_7seg_cc_io.bottom_dot_pin.gpio_pin = DISP_7S_8_Pin;
	display_7seg_cc_io.bottom_dot_pin.gpio_port = DISP_7S_8_GPIO_Port;*/

	display_7seg_cc_io.pin_order[0].gpio_pin = DISP_7S_1_Pin;
	display_7seg_cc_io.pin_order[0].gpio_port = DISP_7S_1_GPIO_Port;
	display_7seg_cc_io.pin_order[1].gpio_pin = DISP_7S_2_Pin;
	display_7seg_cc_io.pin_order[1].gpio_port = DISP_7S_2_GPIO_Port;
	display_7seg_cc_io.pin_order[2].gpio_pin = DISP_7S_3_Pin;
	display_7seg_cc_io.pin_order[2].gpio_port = DISP_7S_3_GPIO_Port;
	display_7seg_cc_io.pin_order[3].gpio_pin = DISP_7S_4_Pin;
	display_7seg_cc_io.pin_order[3].gpio_port = DISP_7S_4_GPIO_Port;
	display_7seg_cc_io.pin_order[4].gpio_pin = DISP_7S_5_Pin;
	display_7seg_cc_io.pin_order[4].gpio_port = DISP_7S_5_GPIO_Port;
	display_7seg_cc_io.pin_order[5].gpio_pin = DISP_7S_6_Pin;
	display_7seg_cc_io.pin_order[5].gpio_port = DISP_7S_6_GPIO_Port;
	display_7seg_cc_io.pin_order[6].gpio_pin = DISP_7S_7_Pin;
	display_7seg_cc_io.pin_order[6].gpio_port = DISP_7S_7_GPIO_Port;
	display_7seg_cc_io.pin_order[7].gpio_pin = DISP_7S_8_Pin;
	display_7seg_cc_io.pin_order[7].gpio_port = DISP_7S_8_GPIO_Port;


	display_7seg_cc_ctor(&display_7seg_cc_io, DISPLAY_7SEG_IO);
	return DISPLAY_7SEG_IO;
}
