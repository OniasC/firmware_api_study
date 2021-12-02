/*
 * 7seg.h
 *
 *  Created on: Aug 2, 2021
 *      Author: onias
 */

#ifndef API_DISPLAY_7_SEG_7SEG_PORT_EXP_H_
#define API_DISPLAY_7_SEG_7SEG_PORT_EXP_H_

#include "main.h"
#include "../states.h"
#include "display_7seg.h"
#include "../portExpansor/portExpansor.h"

/*
 * pin 1 - middle horizantal
 * pin 2 - top left straight
 * pin 3 - top horizontal
 * pin 4 - top right straight
 * pin 5 - bottom left straight
 * pin 6 - bottom horizontal
 * pin 7 - bottom right straight
 * pin 8 - bottom right dot
 *
 * */

typedef struct {
	portExpansor_pinPort_e port;
	uint8_t pinNumber;
} pinMap_t;

typedef struct {
	display_7seg_t super;
	pinMap_t pinMap[8];
	portExpansor_t portExpansor;
	/*DECLARE HERE THE PORT EXPANSOR*/
} display_7seg_portExp_t;

display_7seg_status_e display_7seg_portExp_ctor(display_7seg_portExp_t * const me, portExpansor_t * const portExpansor);

display_7seg_status_e display_7seg_portExp_writeDigitVTable(display_7seg_portExp_t * const me, display_7seg_digit_e number, display_7seg_dot_e dot);

display_7seg_status_e display_7seg_portExp_ZeroVTable(display_7seg_portExp_t * const me, display_7seg_digit_e number);

#endif /* API_DISPLAY_7_SEG_7SEG_PORT_EXP_H_ */
