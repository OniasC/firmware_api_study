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
	EEPROM_ERROR,// = 1,
	EEPROM_ERROR_OVERFLW_PAGS,// = 2,
} eeprom_status;

typedef enum {
	IMU_NO_ERROR = 0,
	IMU_INIT_ERROR = 1,
	IMU_CALIB_ERROR = 2,
} imu_status;

typedef enum {
	DISPLAY_7SEG_NOT_INIT,
	DISPLAY_7SEG_NO_ERROR,
	DISPLAY_7SEG_PORT_EXPANSOR,
	DISPLAY_7SEG_IO,
} display_7seg_status;



#endif /* API_STATES_H_ */
