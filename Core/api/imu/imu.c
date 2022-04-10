/*
 * imu.c
 *
 *  Created on: Jun 9, 2021
 *      Author: onias
 */

#include "imu.h"

static imu_status_e IMU_readAll_VTable(imu_t *imu);
static imu_status_e IMU_readAccel_VTable(imu_t *imu);
static imu_status_e IMU_readGyro_VTable(imu_t *imu);
static imu_status_e IMU_readTemp_VTable(imu_t *imu);

imu_status_e IMU_ctor(imu_t * const imu)
{
	static const struct imu_vtable vtable = {
			&IMU_readAll_VTable,
			&IMU_readAccel_VTable,
			&IMU_readGyro_VTable,
			&IMU_readTemp_VTable,
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

imu_status_e IMU_complementaryFilter(imu_t *imu)
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

imu_status_e IMU_adjustAccelToCM(imu_t *const imu, const float rotationMatrix[][3], float rotationAccel, float displacement[], float *const accelCM, uint8_t numberOfDimensions)
{
	/* imu data needs already to be converted to SI form
	 */
	/* a_cm = a_imu + alfa x d + w x w x d
	 * a_cm : accel of center of mass
	 * a_imu : accel data from IMU
	 * d : distance from IMU and center of mass
	 * w : angular speed
	 * alfa : angular accel
	 *
	 * */

	/* approximations used:
	 *   - only w is in the z axis
	 *   - displacement in z axis is ignored
	 */
	accelCM[0] = rotationMatrix[0][0]*(float)imu->Ax - rotationMatrix[0][1]*(float)imu->Ay + (-imu->Gz*imu->Gz*displacement[0] - rotationAccel*displacement[1]);
	accelCM[1] = rotationMatrix[1][0]*(float)imu->Ax + rotationMatrix[1][1]*(float)imu->Ay + (-imu->Gz*imu->Gz*displacement[1] + rotationAccel*displacement[0]);
	accelCM[2] = (float)imu->Az;

	return imu->status;
}

static imu_status_e IMU_readAll_VTable(imu_t *imu)
{
	(void)imu;
	return IMU_NO_ERROR;
}

static imu_status_e IMU_readAccel_VTable(imu_t *imu)
{
	(void)imu;
	return IMU_NO_ERROR;
}

static imu_status_e IMU_readGyro_VTable(imu_t * const imu)
{
	(void)imu;
	return IMU_NO_ERROR;
}

static imu_status_e IMU_readTemp_VTable(imu_t * const imu)
{
	(void)imu;
	return IMU_NO_ERROR;
}
