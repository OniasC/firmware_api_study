/*
 * api.c
 *
 *  Created on: Aug 1, 2021
 *      Author: onias
 */

#include "api.h"
#include "imu.h"

imu_status imu_error;


void API_Error_Report(error_e *error, bsp_status *status)
{

	if(error != NO_ERROR)
	{
		while (1) {}
	}
}
