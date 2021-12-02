/*
 * display_tft.c
 *
 *  Created on: Sep 23, 2021
 *      Author: onias
 */

#include "tft_display.h"


tft_display_status_e TFT_DISPLAY_ctor(	tft_display_t * const tft_display,
										#ifdef DISPLAY_TFT_USING_USART
											USART_HandleTypeDef *peripheral,
										#endif
										#ifdef DISPLAY_TFT_USING_SPI
											SPI_HandleTypeDef *peripheral,
										#endif
										uint16_t reset_Pin, GPIO_TypeDef * reset_GPIO_Port,
										uint16_t chip_select_Pin, GPIO_TypeDef * chip_select_GPIO_Port,
										TIM_HandleTypeDef *htim, uint32_t channel)
{
	tft_display->reset.gpio_pin = reset_Pin;
	tft_display->reset.gpio_port = reset_GPIO_Port;
	tft_display->chip_select.gpio_pin = chip_select_Pin;
	tft_display->chip_select.gpio_port = chip_select_GPIO_Port;
	tft_display->led_k.htim = htim;
	tft_display->led_k.Channel = channel;
	tft_display->peripheral = peripheral;
	return DISPLAY_TFT_NOT_INIT;
}
