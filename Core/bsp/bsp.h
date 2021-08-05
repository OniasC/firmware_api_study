/*
 * bsp.h
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#ifndef BSP_BSP_H_
#define BSP_BSP_H_

#include "../api/imu.h"
#include "../api/eeprom.h"
#include "../api/mouse.h"
#include "../api/display_7seg.h"

#include "../api/eeprom/at24c.h"
#include "../api/imu/mpu6050.h"
#include "../api/portExpansor/7seg_cc.h"

extern eeprom_at24c eeprom;
extern eeprom_t eeprom2;

extern imu_mpu6050_t imu1;

extern display_7seg_t display_7seg_io;
extern display_7seg_cc_t display_7seg_cc_io;

eeprom_status BSP_EEPROM_Init(void);

imu_status BSP_IMU_Init(void);

display_7seg_status BSP_DISPLAY_7SEG_Init(void);


#endif /* BSP_BSP_H_ */
