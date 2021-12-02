/*
 * 7seg_io.c
 *
 *  Created on: Aug 4, 2021
 *      Author: onias
 */

#include "7seg_io.h"

display_7seg_status_e display_7seg_io_ctor(display_7seg_io_t * const me)
{
	static const struct display_7seg_vtable vtable = {
		(display_7seg_status_e (*)(	display_7seg_t * const me,
									display_7seg_digit_e number,
									display_7seg_dot_e dot))&display_7seg_io_writeDigitVTable,
	};
	display_7seg_status_e display_7seg_ctor_status = display_7seg_ctor(&(me->super));
	me->super.vptr = &vtable;
	return display_7seg_ctor_status;
}

display_7seg_status_e display_7seg_io_writeDigitVTable(display_7seg_io_t * const me, display_7seg_digit_e number, display_7seg_dot_e dot)
{
	display_7seg_io_ZeroVTable(me, disp_7Seg_off);
	for (int i = 0; i < 8U; i++)
	{
		if (((number | dot) & 1<<i) != 0U)
		{
			HAL_GPIO_WritePin(me->pin_order[i].gpio_port, me->pin_order[i].gpio_pin, GPIO_PIN_SET);
		}
		else
			HAL_GPIO_WritePin(me->pin_order[i].gpio_port, me->pin_order[i].gpio_pin, GPIO_PIN_RESET);
	}
	return DISPLAY_7SEG_NO_ERROR;
}

display_7seg_status_e display_7seg_io_ZeroVTable(display_7seg_io_t * const me, display_7seg_digit_e number)
{
	for (int i = 0; i < 8U; i++)
	{
		if ((number & 1<<i) == 1U)
		{
			HAL_GPIO_WritePin(me->pin_order[i].gpio_port, me->pin_order[i].gpio_pin, 1U);
		}
		else
			HAL_GPIO_WritePin(me->pin_order[i].gpio_port, me->pin_order[i].gpio_pin, 0U);
	}
	return DISPLAY_7SEG_NO_ERROR;
}
