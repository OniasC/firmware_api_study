/*
 * states.h
 *
 *  Created on: Aug 3, 2021
 *      Author: onias
 */

#ifndef API_STATES_H_
#define API_STATES_H_

typedef enum {
	EEPROM_NOT_INIT,
	EEPROM_NO_ERROR,// = 0,
	EEPROM_NOT_CONN,
	EEPROM_ERROR,// = 1,
	EEPROM_ERROR_OVERFLW_PAGS,// = 2,
} eeprom_status_e;

typedef enum {
	IMU_NOT_INIT,
	IMU_NO_ERROR,// = 0,
	IMU_NOT_CONN,
	IMU_INIT_ERROR,// = 1,
	IMU_CALIB_ERROR,// = 2,
	IMU_CALL_FUNC_ERROR,
} imu_status_e;

typedef enum {
	DISPLAY_7SEG_NOT_INIT,
	DISPLAY_7SEG_NO_ERROR,
	DISPLAY_7SEG_PORT_EXP,
	DISPLAY_7SEG_PORT_EXP_NOT_CONN,
	DISPLAY_7SEG_IO,
} display_7seg_status_e;

typedef enum {
	DISPLAY_TFT_NOT_INIT = 0,
	DISPLAY_TFT_NO_ERROR,
	DISPLAY_TFT_ERROR
} tft_display_status_e;

typedef enum {
	PORT_EXPANSOR_NOT_INIT = 0,
	PORT_EXPANSOR_NO_ERROR,
	PORT_EXPANSOR_ERROR
}portExpansor_status_e;


typedef enum {
	RADIO_NOT_INIT,
	RADIO_TX_MODE, //is this necessary?
	RADIO_RX_MODE, //is this necessary?
	RADIO_SLEEP_MODE,//is this necessary?
	RADIO_STANDBY_MODE,//is this necessary?
	RADIO_RX_FAILED, //is this necessary?
	RADIO_TX_FAILED,//is this necessary?
	RADIO_ACK_MISSED, //is this necessary?
	RADIO_NO_RECEIVERS_NEARBY,
	RADIO_ERROR,
	RADIO_NO_ERROR,
} radio_status_e; //improve this status enum when reading the datasheet and applications start happening

typedef enum {
	BUZZER_NOT_INIT,
	BUZZER_NO_ERROR,
	BUZZER_ERROR,
} buzzer_status_e;

typedef enum {
	JOYSTICK_NOT_INIT,
	JOYSTICK_NO_ERROR,
	JOYSTICK_ERROR,
} joystick_status_e;

typedef enum {
	MOTOR_NOT_INIT,
	MOTOR_NO_ERROR,
	MOTOR_ERROR,
} motor_status_e;

typedef enum {
	lED_NOT_INIT,
	LED_NO_ERROR,
	LED_ERROR,
} led_status_e;

#endif /* API_STATES_H_ */
