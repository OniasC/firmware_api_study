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

/*
 * how to add a new peripheral to the code:
 * - Create a folder that represents the high level object being created (i.e. imu, eeprom etc)
 * - Inside such folder, create generic .c and .h files that represent all generic attributes of such object
 * - Inside that header file, include "api.h" and "states.h", they hold not only the error functions but also the possible states that peripheral can have
 * 	-- Create a struct that holds all the already mentioned parameters as well as the state
 * 	-- implement in the .h and .c all high level functions
 * - If justified, create specific ICs libraries (.c and .h files) and create a second struct specific for that type of IC
 * 	-- This specific struct shall inherit the functions and parameters of the higher level struct, as one of the parameters of the second struct is the first.
 * 	-- To add polymorphism is a bit trickier and involves created a struct of pointers to functions in order that the high level function can call the lower level
 * 	   function if the parameter is of such type
 *
 * 	example:
 * 		TODO: WRITE DOWN EXAMPLE ...
 *
 * */

/*
 *structure to represent a gpio pin. Params are pin number and port number
 * */
typedef struct {
	uint16_t gpio_pin;
	GPIO_TypeDef *gpio_port;
} io_pin_t;

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
} pwm_t;


typedef struct {
	imu_status_e 			imu_state;
	eeprom_status_e 		eeprom_state;
	display_7seg_status_e 	display_7seg_state;
} bsp_status_t;



#endif /* API_API_H_ */
