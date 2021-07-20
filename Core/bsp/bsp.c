/*
 * bsp.c
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#include "bsp.h"

eeprom_at24c eeprom3;
eeprom_t eeprom2;
imu_mpu6050 imu1;

extern I2C_HandleTypeDef hi2c1;

eeprom_status BSP_EEPROM_Init(void)
{

	eeprom_at24c_ctor(&eeprom3,  EEPROM_AT24C02C, &hi2c1, 0);
	eeprom_ctor(&eeprom2, EEPROM_AT24C02C, &hi2c1, 1);
	return EEPROM_NO_ERROR;
}

imu_status BSP_IMU_Init(void)
{
	return imu_mpu6050_ctor(&imu1, &hi2c1);
}
