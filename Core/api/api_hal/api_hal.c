/*
 * api_hal.c
 *
 *  Created on: 7 Mar 2022
 *      Author: onias
 */

#include"api_hal.h"

__weak void delayMicroseconds(uint32_t uSec)
{
	uint32_t uSecVar = uSec;
	uSecVar = uSecVar* ((SystemCoreClock/1000000)/3);
	while(uSecVar--);
}

__weak void delay(uint32_t Delay)
{
	HAL_Delay(Delay);
}

__weak uint32_t millis(void)
{
	return HAL_GetTick();
}
