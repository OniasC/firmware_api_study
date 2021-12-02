/*
 * tft_lcd.h
 *
 *  Created on: Sep 18, 2021
 *      Author: onias
 */

#ifndef API_DISPLAY_TFT_TFT_DISPLAY_H_
#define API_DISPLAY_TFT_TFT_DISPLAY_H_

#define DISPLAY_TFT_USING_USART 1U
//#define DISPLAY_TFT_USING_SPI 1U
//#define DISPLAY_TFT_USING_DC_PIN 1U

#include "../api.h"
#include "../states.h"

// Color definitions
#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED     0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

typedef enum {
	TFT_ROTATION_0 = 0,
	TFT_ROTATION_90 = 1,
	TFT_ROTATION_180 = 2,
	TFT_ROTATION_270 = 3
} tft_rotation_e;

typedef enum {
	TFT_MODE_USART,
	TFT_MODE_3_PIN_SPI,
	TFT_MODE_4_PIN_SPI,
	TFT_MODE_8bit_PARALLEL,
	TFT_MODE_16bit_PARALLEL
} tft_display_mode_e;



typedef struct {
	int16_t width;       ///< Display width as modified by current rotation
	int16_t height;      ///< Display height as modified by current rotation
	int16_t cursor_x;     ///< x location to start print()ing text
	int16_t cursor_y;     ///< y location to start print()ing text
	tft_rotation_e rotation;     ///< Display rotation (0 thru 3)
	tft_display_mode_e mode;
	tft_display_status_e status;
	io_pin_t reset;
	io_pin_t chip_select;
	pwm_t led_k;
#ifdef DISPLAY_TFT_USING_USART
	USART_HandleTypeDef *peripheral;
#endif
#ifdef DISPLAY_TFT_USING_SPI
	SPI_HandleTypeDef *peripheral;
#endif
#ifdef DISPLAY_TFT_USING_DC_PIN
	io_pin_t dc_pin;
#endif

} tft_display_t;

tft_display_status_e TFT_DISPLAY_ctor(tft_display_t * const tft_display,
										#ifdef DISPLAY_TFT_USING_USART
											USART_HandleTypeDef *peripheral,
										#endif
										#ifdef DISPLAY_TFT_USING_SPI
											SPI_HandleTypeDef *peripheral,
										#endif
										uint16_t reset_Pin, GPIO_TypeDef * reset_GPIO_Port,
										uint16_t chip_select_Pin, GPIO_TypeDef * chip_select_GPIO_Port,
										TIM_HandleTypeDef *htim, uint32_t channel);

#endif /* API_DISPLAY_TFT_TFT_DISPLAY_H_ */
