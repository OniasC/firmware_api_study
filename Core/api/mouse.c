/*
 * mouse.c
 *
 *  Created on: Jun 6, 2021
 *      Author: onias
 */

#include "mouse.h"


void updateCursor(USBD_HandleTypeDef hUsbDeviceFS, mouseHID_t mousehid, int8_t val_x, int8_t val_y)
{
	mousehid.mouse_x = val_x;
	mousehid.mouse_y = val_y;

	USBD_HID_SendReport(&hUsbDeviceFS, &mousehid, sizeof (mousehid));
}

void testFunc1(uint8_t button){
	button++;
}

void testFunc2(uint8_t button){
	button--;
}

