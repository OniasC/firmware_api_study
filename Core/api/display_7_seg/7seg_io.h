/*
 * 7seg.h
 *
 *  Created on: Aug 2, 2021
 *      Author: onias
 */

#ifndef API_DISPLAY_7_SEG_7SEG_IO_H_
#define API_DISPLAY_7_SEG_7SEG_IO_H_

#include "main.h"
#include "../states.h"
#include "display_7seg.h"
/*
 * pin 1 - middle horizantal
 * pin 2 - top left straight
 * pin 3 - vcc
 * pin 4 - top horizontal
 * pin 5 - top right straight
 * pin 6 - bottom left straight
 * pin 7 - bottom horizontal
 * pin 8 - vcc
 * pin 9 - bottom right straight
 * pin 10 - bottom right dot
 *
 * */

typedef struct {
	display_7seg_t super;
	io_pin_t pin_order[8];
	/*display_7seg_io_pin_config_t middle_horizontal_pin;
	display_7seg_io_pin_config_t top_left_vertical_pin;
	display_7seg_io_pin_config_t top_horizontal_pin;
	display_7seg_io_pin_config_t top_right_vertical_pin;
	display_7seg_io_pin_config_t bottom_left_vertical_pin;
	display_7seg_io_pin_config_t bottom_horizontal_pin;
	display_7seg_io_pin_config_t bottom_right_vertical_pin;
	display_7seg_io_pin_config_t bottom_dot_pin;*/
} display_7seg_io_t;

display_7seg_status_e display_7seg_io_ctor(display_7seg_io_t * const me);

display_7seg_status_e display_7seg_io_writeDigitVTable(display_7seg_io_t * const me, display_7seg_digit_e number, display_7seg_dot_e dot);

display_7seg_status_e display_7seg_io_ZeroVTable(display_7seg_io_t * const me, display_7seg_digit_e number);

#endif /* API_DISPLAY_7_SEG_7SEG_IO_H_ */
