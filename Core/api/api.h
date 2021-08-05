/*
 * api.h
 *
 *  Created on: Jun 5, 2021
 *      Author: onias
 */

#ifndef API_API_H_
#define API_API_H_

#include "main.h"
#include "states.h"

#define API_TRUE 1U
#define API_FALSE 0U

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
	ERR_OTHER
} error_e;

typedef struct {
	imu_status 		imu_state;
	eeprom_status 	eeprom_state;
} bsp_status;

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


void API_Error_Report(error_e *error, bsp_status *status);

#endif /* API_API_H_ */
