#include "ST7735.h"

  static const uint16_t
  init_cmds1[] = {            // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   ST7735_DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   ST7735_DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
  init_cmds2[] = {            // Init for 7735R, part 2 (1.44" display)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127
#endif // ST7735_IS_128X128

#ifdef ST7735_IS_160X80
  init_cmds2[] = {            // Init for 7735S, part 2 (160x80 display)
    3,                        //  3 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x4F,             //     XEND = 79
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F ,            //     XEND = 159
    ST7735_INVON, 0 },        //  3: Invert colors
#endif

  init_cmds3[] = {            // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    ST7735_DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    ST7735_DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


// function to reverse bits of a number
uint8_t reverse_byte(uint8_t x)
{
    static const uint8_t table[] = {
        0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
        0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
        0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
        0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
        0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
        0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
        0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
        0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
        0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
        0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
        0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
        0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
        0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
        0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
        0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
        0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
        0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
        0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
        0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
        0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
        0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
        0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
        0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
        0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
        0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
        0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
        0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
        0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
        0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
        0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
        0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
        0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
    };
    return table[x];
}

void ST7735_Select(tft_display_t * const tft_display)
{

    HAL_GPIO_WritePin(tft_display->chip_select.gpio_port, tft_display->chip_select.gpio_pin, GPIO_PIN_RESET);
}

void ST7735_Unselect(tft_display_t * const tft_display)
{
    HAL_GPIO_WritePin(tft_display->chip_select.gpio_port, tft_display->chip_select.gpio_pin, GPIO_PIN_SET);
}

void ST7735_Reset(tft_display_t * const tft_display)
{

    HAL_GPIO_WritePin(tft_display->reset.gpio_port, tft_display->reset.gpio_pin, GPIO_PIN_RESET);
    HAL_Delay(5);
    HAL_GPIO_WritePin(tft_display->reset.gpio_port, tft_display->reset.gpio_pin, GPIO_PIN_SET);
}

void ST7735_WriteCommand(tft_display_t * const tft_display, uint16_t cmd)
{
    uint16_t message = 0U<<8 | cmd;

    message = reverse_byte(cmd) << 1;
    ST7735_Select(tft_display);
    HAL_USART_Transmit(tft_display->peripheral, (uint8_t *)&message, 1, HAL_MAX_DELAY);
    ST7735_Unselect(tft_display);
}

void ST7735_WriteData(tft_display_t * const tft_display, uint16_t* buff, size_t buff_size)
{
    for (int i = 0; i < buff_size; i++)
    {
    	uint16_t message = reverse_byte(buff[i]) << 1 | 1;

    	ST7735_Select(tft_display);
		HAL_USART_Transmit(tft_display->peripheral, (uint8_t *)&message, 1, HAL_MAX_DELAY);
		ST7735_Unselect(tft_display);
    }
}

void ST7735_DisplayInit(tft_display_t * const tft_display, const uint16_t *addr)
{
    uint8_t numCommands, numArgs;
    uint16_t ms;

    numCommands = *addr++;
    while(numCommands--) {
        uint16_t cmd = *addr++;
        ST7735_WriteCommand(tft_display, cmd);

        numArgs = *addr++;
        // If high bit set, delay follows args
        ms = numArgs & ST7735_DELAY;
        numArgs &= ~ST7735_DELAY;
        if(numArgs) {
            ST7735_WriteData(tft_display, (uint16_t*)addr, numArgs);
            addr += numArgs;
        }

        if(ms) {
            ms = *addr++;
            if(ms == 255) ms = 500;
            HAL_Delay(ms);
        }
    }
}

void ST7735_SetAddressWindow(tft_display_t * const tft_display, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
    // column address set
    ST7735_WriteCommand(tft_display, ST7735_CASET);
    uint16_t data[] = { 0x00, (x0 + _xstart), 0x00, (x1 + _xstart) };
    ST7735_WriteData(tft_display, data, sizeof(data));

    // row address set
    ST7735_WriteCommand(tft_display, ST7735_RASET);
    data[1] = (y0 + _ystart);
    data[3] = (y1 + _ystart);
    ST7735_WriteData(tft_display, data, sizeof(data));

    // write to RAM
    ST7735_WriteCommand(tft_display, ST7735_RAMWR);
}

void ST7735_Init(tft_display_t * const tft_display, tft_rotation_e rotation)
{
    ST7735_Select(tft_display);
    ST7735_Reset(tft_display);
    HAL_Delay(100);
    ST7735_DisplayInit(tft_display, init_cmds1);
    ST7735_DisplayInit(tft_display, init_cmds2);
    ST7735_DisplayInit(tft_display, init_cmds3);
#if ST7735_IS_160X80
    _colstart = 24;
    _rowstart = 0;
 /*****  IF Doesn't work, remove the code below (before #elif) *****/
    uint8_t data = 0xC0;
    ST7735_Select();
    ST7735_WriteCommand(ST7735_MADCTL);
    ST7735_WriteData(&data,1);
    ST7735_Unselect();

#elif ST7735_IS_128X128
    _colstart = 2;
    _rowstart = 3;
#else
    _colstart = 0;
    _rowstart = 0;
#endif
    ST7735_SetRotation(tft_display, rotation);
    ST7735_Unselect(tft_display);

}

void ST7735_SetRotation(tft_display_t * const tft_display, tft_rotation_e rotation)
{

  uint16_t madctl = 0;

  //rotation = rotation % 4; // can't be higher than 3

  switch (rotation)
  {
  case 0:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MY | ST7735_MADCTL_RGB;
      tft_display->height = ST7735_HEIGHT;
      tft_display->width = ST7735_WIDTH;
      _xstart = _colstart;
      _ystart = _rowstart;
#endif
    break;
  case 1:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_MY | ST7735_MADCTL_MV | ST7735_MADCTL_RGB;
      tft_display->width = ST7735_HEIGHT;
      tft_display->height = ST7735_WIDTH;
    _ystart = _colstart;
    _xstart = _rowstart;
#endif
    break;
  case 2:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_RGB;
      tft_display->height = ST7735_HEIGHT;
      tft_display->width = ST7735_WIDTH;
    _xstart = _colstart;
    _ystart = _rowstart;
#endif
    break;
  case 3:
#if ST7735_IS_160X80
	  madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_BGR;
#else
      madctl = ST7735_MADCTL_MX | ST7735_MADCTL_MV | ST7735_MADCTL_RGB;
      tft_display->width = ST7735_HEIGHT;
      tft_display->height = ST7735_WIDTH;
    _ystart = _colstart;
    _xstart = _rowstart;
#endif
    break;
  }
  ST7735_Select(tft_display);
  ST7735_WriteCommand(tft_display, ST7735_MADCTL);
  ST7735_WriteData(tft_display, &madctl, 1);
  ST7735_Unselect(tft_display);
}

void ST7735_DrawPixel(tft_display_t * const tft_display, uint16_t x, uint16_t y, uint16_t color) {
    if((x >= tft_display->width) || (y >= tft_display->height))
        return;

    ST7735_Select(tft_display);

    ST7735_SetAddressWindow(tft_display, x, y, x+1, y+1);
    uint16_t data[] = { color >> 8, (color & 0xFF) };
    ST7735_WriteData(tft_display, data, sizeof(data)/2);

    ST7735_Unselect(tft_display);
}

void ST7735_WriteChar(tft_display_t * const tft_display, uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor) {
    uint32_t i, b, j;

    ST7735_SetAddressWindow(tft_display, x, y, x+font.width-1, y+font.height-1);

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                uint16_t data[] = { color >> 8, (color & 0xFF) };
                ST7735_WriteData(tft_display, data, sizeof(data)/2);
            } else {
                uint16_t data[] = { bgcolor >> 8, (bgcolor & 0xFF) };
                ST7735_WriteData(tft_display, data, sizeof(data)/2);
            }
        }
    }
}

void ST7735_WriteString(tft_display_t * const tft_display, uint16_t x, uint16_t y, const char* str, FontDef font, uint16_t color, uint16_t bgcolor) {
    ST7735_Select(tft_display);

    while(*str) {
        if(x + font.width >= tft_display->width) {
            x = 0;
            y += font.height;
            if(y + font.height >= tft_display->height) {
                break;
            }

            if(*str == ' ') {
                // skip spaces in the beginning of the new line
                str++;
                continue;
            }
        }

        ST7735_WriteChar(tft_display, x, y, *str, font, color, bgcolor);
        x += font.width;
        str++;
    }

    ST7735_Unselect(tft_display);
}

void ST7735_FillRectangle(tft_display_t * const tft_display, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    if((x >= tft_display->width) || (y >= tft_display->height)) return;
    if((x + w - 1) >= tft_display->width) w = tft_display->width - x;
    if((y + h - 1) >= tft_display->height) h = tft_display->height - y;
    ST7735_Select(tft_display);
    ST7735_SetAddressWindow(tft_display, x, y, x+w-1, y+h-1);

    uint16_t data[] = {color >> 8, (color & 0xFF) };
    for(y = h; y > 0; y--) {
        for(x = w; x > 0; x--) {
        	for(int i=0; i<2; i++){
        		uint16_t message = reverse_byte(data[i]) << 1 | 1;
        		ST7735_Select(tft_display);
        		HAL_USART_Transmit(tft_display->peripheral, (uint8_t *)&message, 1, HAL_MAX_DELAY);
				ST7735_Unselect(tft_display);
        	}
        }
    }

    ST7735_Unselect(tft_display);
}

void ST7735_DrawImage(tft_display_t * const tft_display, uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t* data)
{
    if((x >= tft_display->width) || (y >= tft_display->height)) return;
    if((x + w - 1) >= tft_display->width) return;
    if((y + h - 1) >= tft_display->height) return;

    ST7735_Select(tft_display);
    ST7735_SetAddressWindow(tft_display, x, y, x+w-1, y+h-1);
    ST7735_WriteData(tft_display, (uint16_t*)data, sizeof(uint16_t)*w*h);
    ST7735_Unselect(tft_display);
}

void ST7735_InvertColors(tft_display_t * const tft_display, bool invert)
{
    ST7735_Select(tft_display);
    ST7735_WriteCommand(tft_display, invert ? ST7735_INVON : ST7735_INVOFF);
    ST7735_Unselect(tft_display);
}

void ST7735_SetBacklight(tft_display_t * const tft_display, uint32_t tft_backlight)
{
	//TODO: check if tft_backlight is higher than max value
	if (tft_display->led_k.Channel == TIM_CHANNEL_1)
		tft_display->led_k.htim->Instance->CCR1 = (uint32_t)tft_backlight;
	else if (tft_display->led_k.Channel == TIM_CHANNEL_2)
		tft_display->led_k.htim->Instance->CCR2 = (uint32_t)tft_backlight;
	else if (tft_display->led_k.Channel == TIM_CHANNEL_3)
			tft_display->led_k.htim->Instance->CCR3 = (uint32_t)tft_backlight;
	else if (tft_display->led_k.Channel == TIM_CHANNEL_4)
			tft_display->led_k.htim->Instance->CCR4 = (uint32_t)tft_backlight;
}


