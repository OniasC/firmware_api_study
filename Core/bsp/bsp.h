/*
 * bsp.h
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#ifndef BSP_BSP_H_
#define BSP_BSP_H_

#include "../api/api.h"

#include "../api/imu.h"
#include "../api/imu/mpu_6050.h"

#include "../api/eeprom.h"
#include "../api/eeprom/at24c.h"

extern eeprom_at24c eeprom3;

extern eeprom_t eeprom2;

extern imu_mpu6050 imu1;

eeprom_status BSP_EEPROM_Init(void);

imu_status BSP_IMU_Init(void);

#endif /* BSP_BSP_H_ */
