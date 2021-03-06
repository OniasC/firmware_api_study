/*
 * bsp.h
 *
 *  Created on: Jul 11, 2021
 *      Author: onias
 */

#ifndef BSP_BSP_H_
#define BSP_BSP_H_

#include "../api/Error_Report.h"
#include "../api/nrf24_radio/layer1_2/MY_NRF24.h"
#include "../api/motor/motor.h"

#include "../api/eeprom/eeprom.h"
#include "../api/eeprom/at24c.h"
#include "../api/eeprom/m24c64.h"

#include "../api/display_7_seg/display_7seg.h"
#include "../api/display_7_seg/7seg_portExp.h"
#include "../api/portExpansor/portExpansor.h"
#include "../api/led/led.h"


#include "../api/imu/imu.h"
#include "../api/imu/mpu6050.h"
#include "../api/imu/mpu9250.h"


#include "../app/robot.h"
#include "../app/control/control.h"
#include "../app/pathPlanning/basicPlanning.h"

#include "../api/sub_mcu/sub_slave.h"
#include "../app/control/kalman_robot.h"

extern led_t led;

extern nrf24_t radio;

extern motor_t motorBrushes;
extern motor_t motorWheelRight;
extern motor_t motorWheelLeft;

extern debug_option_e debug_option;
extern eeprom_m24c64_t eeprom_bsp;
extern eeprom_at24c_t eeprom;

extern imu_mpu6050_t imu1;
extern imu_mpu9250_t brd_imu;

extern mcuSpiSubsriber_t mcuSubs;

extern portExpansor_t portExp;
extern display_7seg_portExp_t disp7SegPExp;

extern robot_t robot;
extern control_pid_t pidWheel[ROBOT_NUM_WHEELS];
extern control_pid_t pidRobot[ROBOT_NUM_SPEEDS];

void BSP_DEBUG_Init(void);

eeprom_status_e BSP_EEPROM_Init(void);

imu_status_e BSP_IMU_Init(void);

tft_display_status_e BSP_DISPLAY_TFT_Init(void);

radio_status_e BSP_Radio_Init_Rx(void);
radio_status_e BSP_Radio_Init_Rx(void);
radio_status_e BSP_Radio_Init(uint8_t channel, uint8_t payloadSize, bool enableAutoACK, nrf24_mode_e mode, bool enableACKPayload, uint64_t address);
display_7seg_status_e BSP_DISPLAY_7SEG_Init(void);
motor_status_e BSP_Motor_Init(void);
motor_status_e BSP_Motor_withEnc_Init(void);

led_status_e BSP_LedGPIO_Init(void);
led_status_e BSP_LedPWM_Init(void);

void BSP_Port_Expansor_Init(void);

void BSP_Robot_Init(void);

#endif /* BSP_BSP_H_ */
