/*
 * imu.c
 *
 *  Created on: Jun 9, 2021
 *      Author: onias
 */

#include "imu.h"

static imu_status IMU_readAllI2C_VTable(imu_t *imu, I2C_HandleTypeDef *hi2c);
static imu_status IMU_readAccelI2C_VTable(imu_t *imu, I2C_HandleTypeDef *hi2c);
static imu_status IMU_readGyroI2C_VTable(imu_t *imu, I2C_HandleTypeDef *I2Cx);
static imu_status IMU_readTempI2C_VTable(imu_t *imu, I2C_HandleTypeDef *I2Cx);

imu_status IMU_I2C_ctor(imu_t * const imu, I2C_HandleTypeDef *hi2c, uint8_t i2c_address_mask)
{
	static const struct imu_vtable vtable = {
			&IMU_readAllI2C_VTable,
			&IMU_readAccelI2C_VTable,
			&IMU_readGyroI2C_VTable,
			&IMU_readTempI2C_VTable
	};
	imu->vptr = &vtable;

	/* Initializing angle values*/
	imu->pitch = 0.0;
	imu->roll = 0.0;

	/* Initializing sensor */
	if (imu->sensor != IMU_MPU_6050)
	{
		return IMU_INIT_ERROR;
	}
	return IMU_NO_ERROR;
}

imu_status IMU_complementaryFilter(imu_t *imu)
{
	float accel_pitch;
	float accel_roll;
	float alpha = (COMP_FILTER_TAU) / (COMP_FILTER_TAU + IMU_SAMPLING_TIME); /*alpha should be around 0.95*/
	// alpha = 0.95;

	imu->pitch += ((float)imu->Gx) * IMU_SAMPLING_TIME; /* Angle around the X-axis */
	imu->roll -= ((float)imu->Gy) * IMU_SAMPLING_TIME;    /* Angle around the Y-axis */

	// Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
	/* Compensate for drift if total acceleration is out of range of sensitivity*/
	int totalAccel = abs((int)imu->Ax) + abs((int)imu->Ay) + abs((int)imu->Az);
	if (totalAccel > 8192 && totalAccel < 32768) /*TODO: replace magic numbers with DEFINEs*/
	{
		/* Turning around the X axis results in a vector on the Y-axis */
		accel_pitch = atan2f((float)imu->Ay, (float)imu->Az) * 180 / M_PI;
		imu->pitch = imu->pitch * (1.0 - alpha) + accel_pitch * alpha;

		// Turning around the Y axis results in a vector on the X-axis
		accel_roll = atan2f((float)imu->Ax, (float)imu->Ax) * 180 / M_PI;
		imu->roll = imu->roll * (1.0 - alpha) + accel_roll * alpha;
	}
	return IMU_NO_ERROR;
}

static imu_status IMU_readAllI2C_VTable(imu_t *imu, I2C_HandleTypeDef *hi2c)
{
	(void)imu;
	return IMU_NO_ERROR;
}

static imu_status IMU_readAccelI2C_VTable(imu_t *imu, I2C_HandleTypeDef *hi2c)
{
	(void)imu;
	return IMU_NO_ERROR;
}

imu_status IMU_readGyroI2C_VTable(imu_t * const imu, I2C_HandleTypeDef *I2Cx)
{
	(void)imu;
	return IMU_NO_ERROR;
}

imu_status IMU_readTempI2C_VTable(imu_t * const imu, I2C_HandleTypeDef *I2Cx)
{
	(void)imu;
	return IMU_NO_ERROR;
}
