/*
 * 7seg_io.c
 *
 *  Created on: Aug 4, 2021
 *      Author: onias
 */

#include "7seg_portExp.h"

display_7seg_status_e display_7seg_portExp_ctor(display_7seg_portExp_t * const me, portExpansor_t * const portExpansor)
{
	static const struct display_7seg_vtable vtable = {
		(display_7seg_status_e (*)(	display_7seg_t * const me,
									display_7seg_digit_e number,
									display_7seg_dot_e dot))&display_7seg_portExp_writeDigitVTable,
	};
	display_7seg_status_e display_7seg_ctor_status = display_7seg_ctor(&(me->super));
	me->portExpansor = *portExpansor;
	me->super.vptr = &vtable;
	return display_7seg_ctor_status;
}

display_7seg_status_e display_7seg_portExp_writeDigitVTable(display_7seg_portExp_t * const me, display_7seg_digit_e number, display_7seg_dot_e dot)
{
	//assuming all leds are in the same port!
	uint8_t bitsHigh = 0U;

	portExpansor_writePins(&(me->portExpansor), port1, 0b11111111, 0);
	for (int i = 0; i < 8U; i++)
	{
		if (((number | dot) & 1<<i) != 0U)
		{
			bitsHigh |= me->pinMap[i].pinNumber;
			//portExpansor_writePins(me->portExpansor, me->pinMap[i].port, me->pinMap[i].pinNumber, 1);
		}
		//else
		//	portExpansor_writePins(me->portExpansor,me->pinMap[i].port, me->pinMap[i].pinNumber, 0);
	}
	portExpansor_writePins(&(me->portExpansor), me->pinMap[0].port, bitsHigh, 1);
	return DISPLAY_7SEG_NO_ERROR;
}

display_7seg_status_e display_7seg_portExp_ZeroVTable(display_7seg_portExp_t * const me, display_7seg_digit_e number)
{
	for (int i = 0; i < 8U; i++)
	{
		if ((number & 1<<i) == 1U)
		{
			portExpansor_writePins(&(me->portExpansor),me->pinMap[i].port, me->pinMap[i].pinNumber, 1);
			//HAL_GPIO_WritePin(me->pin_order[i].gpio_port, me->pin_order[i].gpio_pin, 1U);
		}
		else
			portExpansor_writePins(&(me->portExpansor),me->pinMap[i].port, me->pinMap[i].pinNumber, 0);
			//HAL_GPIO_WritePin(me->pin_order[i].gpio_port, me->pin_order[i].gpio_pin, 0U);
	}
	return DISPLAY_7SEG_NO_ERROR;
}
