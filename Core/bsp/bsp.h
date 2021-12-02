/*
 * bsp.h
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#ifndef BSP_BSP_H_
#define BSP_BSP_H_

#include "../api/Error_Report.h"
#include "../api/tft_display/tft_display.h"
#include "../api/tft_display/ST7735.h"
#include "../api/tft_display/GFX_FUNCTIONS.h"
#include "../api/radio/MY_NRF24.h"
#include "../api/motor/motor.h"
#include "../api/eeprom/eeprom.h"
#include "../api/eeprom/at24c.h"
#include "../api/eeprom/m24c64.h"
#include "../api/display_7_seg/display_7seg.h"
#include "../api/display_7_seg/7seg_portExp.h"
#include "../api/portExpansor/portExpansor.h"

#include "../api/imu/mpu6050.h"
#include "../api/imu/imu.h"


extern tft_display_t tft_display;
extern nrf24_t radio;
extern motor_t motor1;
extern motor_t motor_enc;
extern debug_option_e debug_option;
extern eeprom_at24c_t eeprom;
extern eeprom_t eeprom2;
extern eeprom_m24c64_t eeprom_bsp;
extern imu_mpu6050_t imu1;
extern portExpansor_t portExp;
extern display_7seg_portExp_t disp7SegPExp;

void BSP_DEBUG_Init(void);

eeprom_status_e BSP_EEPROM_Init(void);

imu_status_e BSP_IMU_Init(void);

tft_display_status_e BSP_DISPLAY_TFT_Init(void);

radio_status_e BSP_Radio_Init_Rx(void);
radio_status_e BSP_Radio_Init_Tx(void);
display_7seg_status_e BSP_DISPLAY_7SEG_Init(void);
motor_status_e BSP_Motor_Init(void);
motor_status_e BSP_Motor_w_Enc_Init(void);

void BSP_Port_Expansor_Init(void);

#endif /* BSP_BSP_H_ */
