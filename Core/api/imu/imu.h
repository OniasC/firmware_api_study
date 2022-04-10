/*
 * imu.h
 *
 *  Created on: Jun 9, 2021
 *      Author: onias
 */

#ifndef API_IMU_H_
#define API_IMU_H_

#include "main.h"
#include "../api.h"
#include "math.h"
#include "../states.h"

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
	imu_status_e status;
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
	imu_status_e (*IMU_readAllVCall)(imu_t * const imu);
	imu_status_e (*IMU_readAccelVCall)(imu_t * const imu);
	imu_status_e (*IMU_readGyroVCall)(imu_t * const imu);
	imu_status_e (*IMU_readTempVCall)(imu_t * const imu);
};

imu_status_e IMU_ctor(imu_t * const imu);
imu_status_e IMU_complementaryFilter(imu_t *imu);

imu_status_e IMU_adjustAccelToCM(imu_t *const imu, const float rotationMatrix[][3], float rotationAccel, float displacement[], float *const accelCM, uint8_t numberOfDimensions);

/*virtual calls (late bindings) */
static inline imu_status_e IMU_readAll(imu_t *imu)
{
	return (*imu->vptr->IMU_readAllVCall)(imu);
}

static inline imu_status_e IMU_readAccel(imu_t *imu)
{
	return (*imu->vptr->IMU_readAccelVCall)(imu);
}

static inline imu_status_e IMU_readGyro(imu_t *imu)
{
	return (*imu->vptr->IMU_readGyroVCall)(imu);
}

static inline imu_status_e IMU_readTemp(imu_t *imu)
{
	return (*imu->vptr->IMU_readTempVCall)(imu);
}


#endif /* API_IMU_H_ */
