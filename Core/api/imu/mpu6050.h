/*
 * mpu_6050.h
 *
 *  Created on: Jul 19, 2021
 *      Author: onias
 */

#ifndef API_IMU_MPU6050_H_
#define API_IMU_MPU6050_H_

#include "imu.h"

#define RAD_TO_DEG 57.295779513082320876798154814105

#define MPU6050_WHO_AM_I_REG 0x75
#define MPU6050_PWR_MGMT_1_REG 0x6B
#define MPU6050_SMPLRT_DIV_REG 0x19
#define MPU6050_ACCEL_CONFIG_REG 0x1C
#define MPU6050_ACCEL_XOUT_H_REG 0x3B
#define MPU6050_TEMP_OUT_H_REG 0x41
#define MPU6050_GYRO_CONFIG_REG 0x1B
#define MPU6050_GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0


uint32_t timer;

typedef struct {
	imu_t imu;
	uint16_t i2c_timeout;// = 100;
	double Accel_Z_corrector;// = 14418.0;
	uint8_t i2c_address;
	I2C_HandleTypeDef *hi2c;
} imu_mpu6050_t;

imu_status_e imu_mpu6050_ctor(imu_mpu6050_t * const mpu6050, I2C_HandleTypeDef *hi2c);

imu_status_e imu_mpu6050_readAccelVTable(imu_mpu6050_t * const mpu6050);

imu_status_e imu_mpu6050_readGyroVTable(imu_mpu6050_t * const mpu6050);

imu_status_e imu_mpu6050_readTempVTable(imu_mpu6050_t * const mpu6050);

imu_status_e imu_mpu6050_readAllVTable(imu_mpu6050_t * const mpu6050);


#endif /* API_IMU_MPU6050_H_ */
