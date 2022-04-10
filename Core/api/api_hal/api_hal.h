/*
 * api_hal.h
 *
 *  Created on: 7 Mar 2022
 *      Author: onias
 */

#ifndef API_API_HAL_API_HAL_H_
#define API_API_HAL_API_HAL_H_


#include"../api.h"

__weak void delayMicroseconds(uint32_t uSec);
__weak void delay(uint32_t Delay);
__weak uint32_t millis(void);
#endif /* API_API_HAL_API_HAL_H_ */
