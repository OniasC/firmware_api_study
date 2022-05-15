/*
 * lcd_lvgl.h
 *
 *  Created on: May 2, 2022
 *      Author: onias
 */

#ifndef APP_LVGL_TFT_LAYER_LCD_LVGL_H_
#define APP_LVGL_TFT_LAYER_LCD_LVGL_H_


/*********************
 *      INCLUDES
 *********************/
#include <stdint.h>
#include "../thirdparties/lvgl-8.2.0/lvgl.h"

/*********************
 *      DEFINES
 *********************/
#define DISP_HOR_RES 128
#define DISP_VER_RES 160


/**********************
 * GLOBAL PROTOTYPES
 **********************/
void Display_init(int rotation);




#endif /* APP_LVGL_TFT_LAYER_LCD_LVGL_H_ */
