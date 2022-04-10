/*
 * Error_Report.h
 *
 *  Created on: Oct 9, 2021
 *      Author: onias
 */

#ifndef API_ERROR_REPORT_H_
#define API_ERROR_REPORT_H_


//#define SERIAL_DEBUG_MINIMAL

#include "api.h"

#define API_DEBUG_MESSAGE_2(...)            					\
		sprintf(DEBUG_STRING, __VA_ARGS__); 					\
	  	API_Debug_Messages(DEBUG_STRING, sizeof(DEBUG_STRING)); \
		memset(DEBUG_STRING, 0, strlen(DEBUG_STRING))			\

#define API_DEBUG_MESSAGE(x) 			 \
		API_Debug_Messages(x, strlen(x));\
		memset(x, 0, strlen(x))			 \

#define API_ERROR_REPORT(error, level, x) API_Error_Report(error, level, x, strlen(x))
/*
 * !! How I want api reporting to work:
 *
 * If an interface to log or show the error is available (sdcard, lcd screen, 7seg display, leds)
 * it will receive the error command and a few extra informations (like what was expected/received, etc)
 * But it should not stop the flow of the code (it would, depending if the error is big or small)
 *
 * it would get the status of the sensor (whether it would be imu, eeprom, lcd, sd card, whatever)
 * It should be called in the function the error appeared but also the function returns the sensor status if
 * less overhead is desired and not so intense error checking
 *
 * error_e could be filled with all the errors to be more a succint way to report the errors
 * */
#define DEBUG_STRING_BUFFER_SIZE 256
char DEBUG_STRING[256];

typedef enum {
	NO_ERROR,
	ERR_IMU,
	ERR_EEPROM,
	ERR_USB_HID,
	ERR_SD_CARD,
	ERR_LCD,
	ERR_IR,
	ERR_MOTOR_0,
	ERR_MOTOR_1,
	ERR_OTHER,
	ERR_RADIO,
	ERR_PORT_EXP,
	ERR_I2C
} error_e;

typedef enum {
	ERR_LVL_LOG,
	ERR_LVL_WARNING,
	ERR_LVL_ERROR,
	ERR_LVL_CRITICAL,
	ERR_LVL_MISSING
} error_level_e;

typedef enum {
	DEBUG_NONE = 0x0,
	DEBUG_PC_UART = 0x1,
	DEBUG_PC_USB_COM = 0x2, //not implemented
	DEBUG_TFT_LCD = 0x4,
	DEBUG_7_SEG_DISPLAY = 0x8,
	DEBUG_LEDS = 0x10,
	DEBUG_SD_CARD = 0x20
	//Unknown option values 0x40
	//Unknown option values 0x80
} debug_option_e;

extern debug_option_e debug_option;

extern UART_HandleTypeDef huart1;

void API_Debug_Messages(const char *error_message, uint16_t len);

void API_Error_Report(error_e error, error_level_e level, const char *error_message, uint16_t len);

void API_Error_Show(error_e *error, bsp_status_t status, error_level_e level);

void API_Error_Log(error_e *error, bsp_status_t status, error_level_e level);

#endif /* API_ERROR_REPORT_H_ */
