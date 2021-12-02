/*
 * GFX_FUNCTIONS.h
 *
 *  Created on: 30-Oct-2020
 *      Author: meh
 */


#ifndef INC_GFX_FUNCTIONS_H_
#define INC_GFX_FUNCTIONS_H_


#include "ST7735.h"
#include "stdint.h"
#include "stdlib.h"

#define _swap_int16_t(a, b)                                                    \
  {                                                                            \
    int16_t t = a;                                                             \
    a = b;                                                                     \
    b = t;                                                                     \
  }

#define min(a, b) (((a) < (b)) ? (a) : (b))


void drawPixel(tft_display_t * const tft_display, int16_t x, int16_t y, uint16_t color);
void writeLine(tft_display_t * const tft_display, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void drawFastVLine(tft_display_t * const tft_display, int16_t x, int16_t y, int16_t h, uint16_t color);
void drawFastHLine(tft_display_t * const tft_display, int16_t x, int16_t y, int16_t w, uint16_t color);
void fillRect(tft_display_t * const tft_display, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void drawLine(tft_display_t * const tft_display, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void drawCircle(tft_display_t * const tft_display, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void drawCircleHelper(tft_display_t * const tft_display, int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void fillCircleHelper(tft_display_t * const tft_display, int16_t x0, int16_t y0, int16_t r, uint8_t corners, int16_t delta, uint16_t color);
void fillCircle(tft_display_t * const tft_display, int16_t x0, int16_t y0, int16_t r, uint16_t color);
void drawRect(tft_display_t * const tft_display, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void drawRoundRect(tft_display_t * const tft_display, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void fillRoundRect(tft_display_t * const tft_display, int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void drawTriangle(tft_display_t * const tft_display, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void fillTriangle(tft_display_t * const tft_display, int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void fillScreen(tft_display_t * const tft_display, uint16_t color);
void testLines(tft_display_t * const tft_display, uint16_t color);
void testFastLines(tft_display_t * const tft_display, uint16_t color1, uint16_t color2);
void testRects(tft_display_t * const tft_display, uint16_t color) ;
void testFilledRects(tft_display_t * const tft_display, uint16_t color1, uint16_t color2);
void testFilledCircles(tft_display_t * const tft_display, uint8_t radius, uint16_t color);
void testCircles(tft_display_t * const tft_display, uint8_t radius, uint16_t color);
void testTriangles(tft_display_t * const tft_display);
void testFilledTriangles(tft_display_t * const tft_display);
void testRoundRects(tft_display_t * const tft_display);
void testFilledRoundRects(tft_display_t * const tft_display);
void testFillScreen(tft_display_t * const tft_display);
void testAll(tft_display_t * const tft_display);

#endif /* INC_GFX_FUNCTIONS_H_ */
