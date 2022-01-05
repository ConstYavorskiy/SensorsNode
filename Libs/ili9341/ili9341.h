/* vim: set ai et ts=4 sw=4: */
#ifndef __ILI9341_H__
#define __ILI9341_H__

#include "main.h"
#include "fonts.h"
#include <stdbool.h>

#define ILI9341_MADCTL_MY  0x80
#define ILI9341_MADCTL_MX  0x40
#define ILI9341_MADCTL_MV  0x20
#define ILI9341_MADCTL_ML  0x10
#define ILI9341_MADCTL_RGB 0x00
#define ILI9341_MADCTL_BGR 0x08
#define ILI9341_MADCTL_MH  0x04

/*** Redefine if necessary ***/
#define ILI9341_SPI_PORT hspi2
extern SPI_HandleTypeDef ILI9341_SPI_PORT;

#define ILI9341_RES_Pin       GPIO_PIN_10
#define ILI9341_RES_GPIO_Port GPIOA
#define ILI9341_CS_Pin        GPIO_PIN_12
#define ILI9341_CS_GPIO_Port  GPIOB
#define ILI9341_DC_Pin        GPIO_PIN_9
#define ILI9341_DC_GPIO_Port  GPIOA

// default orientation
/*
#define ILI9341_WIDTH  240
#define ILI9341_HEIGHT 320
#define ILI9341_ROTATION (ILI9341_MADCTL_MX | ILI9341_MADCTL_BGR)
*/
// rotate right
/*
 #define ILI9341_WIDTH  320
 #define ILI9341_HEIGHT 240
 #define ILI9341_ROTATION (ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR)
 */

// rotate left
/*
 #define ILI9341_WIDTH  320
 #define ILI9341_HEIGHT 240
 #define ILI9341_ROTATION (ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR)
 */

// upside down
 #define ILI9341_WIDTH  240
 #define ILI9341_HEIGHT 320
 #define ILI9341_ROTATION (ILI9341_MADCTL_MY | ILI9341_MADCTL_BGR)

/****************************/

// Color definitions
#define BLACK       0x0000      /*   0,   0,   0 */
#define NAVY        0x000F      /*   0,   0, 128 */
#define DARKGREEN   0x03E0      /*   0, 128,   0 */
#define DARKCYAN    0x03EF      /*   0, 128, 128 */
#define MAROON      0x7800      /* 128,   0,   0 */
#define PURPLE      0x780F      /* 128,   0, 128 */
#define OLIVE       0x7BE0      /* 128, 128,   0 */
#define LIGHTGREY   0xC618      /* 192, 192, 192 */
#define DARKGREY    0x7BEF      /* 128, 128, 128 */
#define BLUE        0x001F      /*   0,   0, 255 */
#define GREEN       0x07E0      /*   0, 255,   0 */
#define CYAN        0x07FF      /*   0, 255, 255 */
#define RED         0xF800      /* 255,   0,   0 */
#define MAGENTA     0xF81F      /* 255,   0, 255 */
#define YELLOW      0xFFE0      /* 255, 255,   0 */
#define WHITE       0xFFFF      /* 255, 255, 255 */
#define ORANGE      0xFD20      /* 255, 165,   0 */
#define GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define PINK        0xF81F
#define COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

// call before initializing any SPI devices
void ILI9341_Unselect();

void ILI9341_Init(void);
void ILI9341_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ILI9341_WriteInt(uint16_t x, uint16_t y, int32_t nummber, uint8_t placeholders, FontDef font, uint16_t color, uint16_t bgcolor);
void ILI9341_WriteUInt(uint16_t x, uint16_t y, uint16_t nummber, uint8_t placeholders, FontDef font, uint16_t color, uint16_t bgcolor);
void ILI9341_WriteFloat(uint16_t x, uint16_t y, float nummber, uint8_t decimals, uint8_t placeholders, FontDef font, uint16_t color, uint16_t bgcolor);
void ILI9341_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor);
void ILI9341_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_FillScreen(uint16_t color);
void ILI9341_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
void ILI9341_InvertColors(bool invert);

#endif // __ILI9341_H__
