/*
 * mouse.h
 *
 *  Created on: Jun 6, 2021
 *      Author: onias
 */

#ifndef API_MOUSE_H_
#define API_MOUSE_H_

#include "main.h"
#include "usbd_hid.h"

typedef struct
{
	uint8_t button;
	int8_t mouse_x;
	int8_t mouse_y;
	int8_t wheel;
	void (*testFunc)(uint8_t button);
} mouseHID_t;

void testFunc1(uint8_t button);

void testFunc2(uint8_t button);

void updateCursor(USBD_HandleTypeDef hUsbDeviceFS, mouseHID_t mousehid, int8_t val_x, int8_t val_y);

#endif /* API_MOUSE_H_ */
