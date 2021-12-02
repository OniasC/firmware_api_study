#ifndef __ST7735_H__
#define __ST7735_H__

#include "fonts.h"
#include "tft_display.h"
#include <stdbool.h>


/****** TFT DEFINES ******/
//#define ST7735_IS_160X80 1
//#define ST7735_IS_128X128 1
#define ST7735_IS_160X128 1
#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160

#define ST7735_DELAY 0x80

#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_ML  0x10
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1


#define color565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))


// call before initializing any SPI devices
void ST7735_Unselect(tft_display_t * const tft_display);
void ST7735_Select(tft_display_t * const tft_display);
void ST7735_Reset(tft_display_t * const tft_display);

void ST7735_SetAddressWindow(tft_display_t * const tft_display, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ST7735_WriteData(tft_display_t * const tft_display, uint16_t* buff, size_t buff_size);
void ST7735_WriteCommand(tft_display_t * const tft_display, uint16_t cmd);
void ST7735_DisplayInit(tft_display_t * const tft_display, const uint16_t *addr);
void ST7735_Init(tft_display_t * const tft_display, tft_rotation_e rotation);
void ST7735_SetRotation(tft_display_t * const tft_display, tft_rotation_e rotation);
void ST7735_DrawPixel(tft_display_t * const tft_display, uint16_t x, uint16_t y, uint16_t color);
void ST7735_WriteChar(tft_display_t * const tft_display, uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7735_WriteString(tft_display_t * const tft_display, uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor);
void ST7735_FillRectangle(tft_display_t * const tft_display, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
//void ST7735_FillScreen(uint16_t color);
void ST7735_DrawImage(tft_display_t * const tft_display, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data);
void ST7735_InvertColors(tft_display_t * const tft_display, bool invert);

void ST7735_SetBacklight(tft_display_t * const tft_display, uint32_t tft_backlight);

//int16_t _width;       ///< Display width as modified by current rotation
//int16_t _height;      ///< Display height as modified by current rotation
int16_t cursor_x;     ///< x location to start print()ing text
int16_t cursor_y;     ///< y location to start print()ing text
//uint8_t rotation;     ///< Display rotation (0 thru 3)
uint8_t _colstart;   ///< Some displays need this changed to offset
uint8_t _rowstart;       ///< Some displays need this changed to offset
uint8_t _xstart;
uint8_t _ystart;

#endif // __ST7735_H__
