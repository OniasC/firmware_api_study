/*
 * display_7seg.c
 *
 *  Created on: Aug 4, 2021
 *      Author: onias
 */

#include "display_7seg.h"

static display_7seg_status_e display_7seg_WriteDigitVTable(display_7seg_t * const me, display_7seg_digit_e number, display_7seg_dot_e dot);

display_7seg_status_e display_7seg_ctor(display_7seg_t * const me)
{
	static const struct display_7seg_vtable vtable = {
		&display_7seg_WriteDigitVTable
	};
	me->vptr = &vtable;

	return DISPLAY_7SEG_NOT_INIT;
}

static display_7seg_status_e display_7seg_WriteDigitVTable(display_7seg_t * const me, display_7seg_digit_e number, display_7seg_dot_e dot)
{
	(void)me;
	return DISPLAY_7SEG_NO_ERROR;//check if code ends up in this line.
}
