/*
 * mpu_6050.c
 *
 *  Created on: Jul 19, 2021
 *      Author: onias
 */

#include "mpu6050.h"


imu_status_e imu_mpu6050_ctor(imu_mpu6050_t * const mpu6050, I2C_HandleTypeDef *hi2c)
{
	static const struct imu_vtable vtable = {
			(imu_status_e (*)(	imu_t * const mpu6050))&imu_mpu6050_readAllVTable,
			(imu_status_e (*)(	imu_t * const mpu6050))&imu_mpu6050_readAccelVTable,
			(imu_status_e (*)(	imu_t * const mpu6050))&imu_mpu6050_readGyroVTable,
			(imu_status_e (*)(	imu_t * const mpu6050))&imu_mpu6050_readTempVTable,
		};
		imu_status_e imu_ctor_status = IMU_ctor(&(mpu6050->imu));
		mpu6050->imu.vptr = &vtable;
		mpu6050->i2c_timeout = 100;
		mpu6050->Accel_Z_corrector = 14418.0;
		mpu6050->imu.status = IMU_NOT_INIT;
		return imu_ctor_status;
}

imu_status_e imu_mpu6050_readAccelVTable(imu_mpu6050_t * const mpu6050)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read(mpu6050->hi2c, mpu6050->i2c_address, MPU6050_ACCEL_XOUT_H_REG, 1, Rec_Data, 6, mpu6050->i2c_timeout);
	HAL_Delay(50);

	mpu6050->imu.Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	mpu6050->imu.Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	mpu6050->imu.Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	/*** convert the RAW values into acceleration in 'g'
		 we have to divide according to the Full scale value set in FS_SEL
		 I have configured FS_SEL = 0. So I am dividing by 16384.0
		 for more details check ACCEL_CONFIG Register              ****/

	mpu6050->imu.Ax = mpu6050->imu.Accel_X_RAW / 16384.0;
	mpu6050->imu.Ay = mpu6050->imu.Accel_Y_RAW / 16384.0;
	mpu6050->imu.Az = mpu6050->imu.Accel_Z_RAW / mpu6050->Accel_Z_corrector;

	return IMU_NO_ERROR;
}

imu_status_e imu_mpu6050_readGyroVTable(imu_mpu6050_t * const mpu6050)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read(mpu6050->hi2c, mpu6050->i2c_address, MPU6050_GYRO_XOUT_H_REG, 1, Rec_Data, 6, mpu6050->i2c_timeout);
	HAL_Delay(50);

	mpu6050->imu.Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	mpu6050->imu.Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	mpu6050->imu.Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

	/*** convert the RAW values into dps (degrees/s)
			 we have to divide according to the Full scale value set in FS_SEL
			 I have configured FS_SEL = 0. So I am dividing by 131.0
			 for more details check GYRO_CONFIG Register                 ****/

	mpu6050->imu.Gx = mpu6050->imu.Gyro_X_RAW / 131.0;
	mpu6050->imu.Gy = mpu6050->imu.Gyro_Y_RAW / 131.0;
	mpu6050->imu.Gz = mpu6050->imu.Gyro_Z_RAW / 131.0;

	return IMU_NO_ERROR;
}

imu_status_e imu_mpu6050_readTempVTable(imu_mpu6050_t * const mpu6050)
{
	uint8_t Rec_Data[2];
	int16_t temp;

	// Read 2 BYTES of data starting from TEMP_OUT_H_REG register

	HAL_I2C_Mem_Read(mpu6050->hi2c, mpu6050->i2c_address, MPU6050_TEMP_OUT_H_REG, 1, Rec_Data, 2, mpu6050->i2c_timeout);

	temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	mpu6050->imu.Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	return IMU_NO_ERROR;
}

imu_status_e imu_mpu6050_readAllVTable(imu_mpu6050_t * const mpu6050)
{
	uint8_t Rec_Data[14];
	int16_t temp;

	// Read 14 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read(mpu6050->hi2c, mpu6050->i2c_address, MPU6050_ACCEL_XOUT_H_REG, 1, Rec_Data, 14, mpu6050->i2c_timeout);

	mpu6050->imu.Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	mpu6050->imu.Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	mpu6050->imu.Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
	mpu6050->imu.Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
	mpu6050->imu.Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
	mpu6050->imu.Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

	mpu6050->imu.Ax = mpu6050->imu.Accel_X_RAW / 16384.0;
	mpu6050->imu.Ay = mpu6050->imu.Accel_Y_RAW / 16384.0;
	mpu6050->imu.Az = mpu6050->imu.Accel_Z_RAW / mpu6050->Accel_Z_corrector;
	mpu6050->imu.Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
	mpu6050->imu.Gx = mpu6050->imu.Gyro_X_RAW / 131.0;
	mpu6050->imu.Gy = mpu6050->imu.Gyro_Y_RAW / 131.0;
	mpu6050->imu.Gz = mpu6050->imu.Gyro_Z_RAW / 131.0;

	return IMU_NO_ERROR;
}

