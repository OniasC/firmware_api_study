/*
 * imu.h
 *
 *  Created on: Jun 9, 2021
 *      Author: onias
 */

#ifndef API_IMU_H_
#define API_IMU_H_

#include "main.h"
#include "api.h"
#include "math.h"
#include "states.h"

#define PI 3.14159265359
#define IMU_SAMPLING_TIME 0.01
#define COMP_FILTER_TAU 1900

typedef enum {
	IMU_MPU_6050,
	IMU_MPU_9250
	/* Enter later other IMUs we could end up using */
} imu_sensor;



typedef struct
{
	struct imu_vtable const *vptr; /* virtual pointer */
	imu_sensor sensor;
	imu_status status;
    float pitch;
    float roll;

    /* Parameters shared with other structures*/
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;
    int16_t test;

} imu_t;

struct imu_vtable {
	imu_status (*IMU_readAllI2CVCall)(imu_t * const imu, I2C_HandleTypeDef *hi2c);
	imu_status (*IMU_readAccelI2CVCall)(imu_t * const imu, I2C_HandleTypeDef *hi2c);
	imu_status (*IMU_readGyroI2CVCall)(imu_t * const imu, I2C_HandleTypeDef *I2Cx);
	imu_status (*IMU_readTempI2CVCall)(imu_t * const imu, I2C_HandleTypeDef *I2Cx);
};

imu_status IMU_I2C_ctor(imu_t * const imu, I2C_HandleTypeDef *hi2c, uint8_t i2c_address_mask);
imu_status IMU_complementaryFilter(imu_t *imu);

/*virtual calls (late bindings) */
static inline imu_status IMU_readAllI2C(imu_t *imu, I2C_HandleTypeDef *hi2c)
{
	return (*imu->vptr->IMU_readAllI2CVCall)(imu, hi2c);
}

static inline imu_status IMU_readAccelI2C(imu_t *imu, I2C_HandleTypeDef *hi2c)
{
	return (*imu->vptr->IMU_readAccelI2CVCall)(imu, hi2c);
}

static inline imu_status IMU_readGyroI2C(imu_t *imu, I2C_HandleTypeDef *I2Cx)
{
	return (*imu->vptr->IMU_readGyroI2CVCall)(imu, I2Cx);
}

static inline imu_status IMU_readTempI2C(imu_t *imu, I2C_HandleTypeDef *I2Cx)
{
	return (*imu->vptr->IMU_readTempI2CVCall)(imu, I2Cx);
}


#endif /* API_IMU_H_ */
