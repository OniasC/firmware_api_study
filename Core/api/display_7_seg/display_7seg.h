/*
 * display_7seg.h
 *
 *  Created on: Aug 4, 2021
 *      Author: onias
 */

#ifndef API_DISPLAY_7SEG_H_
#define API_DISPLAY_7SEG_H_

#include "../states.h"
#include "../api.h"
#include "main.h"

typedef enum {
	middle_horizontal		= 0x01,
	top_left_vertical		= 0x02,
	top_horizontal			= 0x04,
	top_right_vertical		= 0x08,
	bottom_left_vertical	= 0x10,
	bottom_horizontal		= 0x20,
	bottom_right_vertical	= 0x40,
	bottom_dot				= 0x80
} display_7seg_leds_e;

typedef enum {
	disp_7Seg_off = 0x0U,
	disp_7Seg_0 = top_horizontal | bottom_horizontal | top_left_vertical | bottom_left_vertical | top_right_vertical | bottom_right_vertical,
	disp_7Seg_1 = top_right_vertical | bottom_right_vertical,
	disp_7Seg_2 = top_horizontal | middle_horizontal | bottom_horizontal | bottom_left_vertical | top_right_vertical,
	disp_7seg_3 = top_horizontal | bottom_horizontal | middle_horizontal | top_right_vertical | bottom_right_vertical,
	disp_7Seg_4 = top_left_vertical | middle_horizontal | top_right_vertical | bottom_right_vertical,
	disp_7Seg_5 = top_horizontal | top_left_vertical | middle_horizontal | bottom_right_vertical | bottom_horizontal,
	disp_7Seg_6 = top_horizontal | top_left_vertical | middle_horizontal | bottom_right_vertical | bottom_left_vertical | bottom_horizontal,
	disp_7Seg_7 = top_horizontal | top_right_vertical | bottom_right_vertical,
	disp_7Seg_8 = top_horizontal | bottom_horizontal | middle_horizontal | top_left_vertical | bottom_left_vertical | top_right_vertical | bottom_right_vertical,
	disp_7Seg_9 = top_horizontal | bottom_horizontal | middle_horizontal | top_left_vertical | top_right_vertical | bottom_right_vertical,
	disp_7Seg_a = top_horizontal | middle_horizontal | top_left_vertical | bottom_left_vertical | top_right_vertical | bottom_right_vertical,
	disp_7Seg_b = top_left_vertical | middle_horizontal | bottom_right_vertical | bottom_left_vertical | bottom_horizontal,
	disp_7Seg_c = top_horizontal | bottom_horizontal | top_left_vertical | bottom_left_vertical,
	disp_7Seg_d = bottom_horizontal | middle_horizontal | top_right_vertical | bottom_right_vertical | bottom_left_vertical,
	disp_7Seg_e = top_horizontal | bottom_horizontal | middle_horizontal | top_left_vertical | bottom_left_vertical,
	disp_7Seg_f = top_horizontal | middle_horizontal | top_left_vertical | bottom_left_vertical
} display_7seg_digit_e;

typedef enum {
	dot_on = bottom_dot,
	dot_off = 0x0U,
} display_7seg_dot_e;

typedef struct {
	struct display_7seg_vtable const *vptr; /*virtual pointer*/

	uint16_t refresh_freq;
	display_7seg_digit_e digit;
	display_7seg_dot_e dot;
	display_7seg_status_e status;
} display_7seg_t;

struct display_7seg_vtable {
	display_7seg_status_e (*display_7seg_WriteDigitVCall)(display_7seg_t * const me, display_7seg_digit_e number, display_7seg_dot_e dot);
};

display_7seg_status_e display_7seg_ctor(display_7seg_t * const me);

static inline display_7seg_status_e display_7seg_WriteDigit(display_7seg_t * const me, display_7seg_digit_e number, display_7seg_dot_e dot)
{
	return (*me->vptr->display_7seg_WriteDigitVCall)(me, number, dot);
}

#endif /* API_DISPLAY_7SEG_H_ */
