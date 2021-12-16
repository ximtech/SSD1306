#pragma once

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "I2C_Polling.h"

// Display resolution
#ifndef SSD1306_RESOLUTION_WIDTH
#define SSD1306_RESOLUTION_WIDTH 128
#endif

#ifndef SSD1306_RESOLUTION_HEIGHT
#define SSD1306_RESOLUTION_HEIGHT 32
#endif

#ifndef SSD1306_I2C_TIMEOUT_MS
#define SSD1306_I2C_TIMEOUT_MS 100
#endif

#ifndef SD1306_DEVICE_ADDRESS
#define SD1306_DEVICE_ADDRESS 0x78 // Notice: SSD1306 128x32 can only have one I2C address
#endif

// Font define section. Uncomment/comment for font enable or disable
//#define SSD1306_FONT_DEFAULT  1

#ifdef SSD1306_FONT_DEFAULT
#include "Fonts/SSD1306_Font_Default_128x32.h"
#endif

#define SSD1306_SET_COLUMN_ADDRESS_COMMAND 0x21
#define SSD1306_SET_PAGE_ADDRESS_COMMAND 0x22

#define SSD1306_DISPLAY_ON_COMMAND 0xAF
#define SSD1306_DISPLAY_OFF_COMMAND 0xAE
#define SSD1306_NORMAL_DISPLAY_COMMAND 0xA6
#define SSD1306_INVERT_DISPLAY_COMMAND 0xA7

#define SSD1306_SET_DISPLAY_CLOCK_DIVISION_COMMAND 0xD5
#define SSD1306_SET_MULTIPLEX_COMMAND 0xA8
#define SSD1306_SET_DISPLAY_OFFSET_COMMAND 0xD3
#define SSD1306_SET_START_LINE_COMMAND 0x40
#define SSD1306_CHARGE_PUMP_COMMAND 0x8D
#define SSD1306_SET_MEMORY_MODE_COMMAND 0x20
#define SSD1306_SEGRE_MAP_COMMAND 0xA0
#define SSD1306_COMMUNICATION_SCAN_DETECTED_COMMAND 0xC8
#define SSD1306_SET_COM_PINS_COMMAND 0xDA
#define SSD1306_SET_CONTRAST_COMMAND 0x81
#define SSD1306_SET_PRE_CHARGE_COMMAND 0xD9
#define SSD1306_SET_V_COM_DETECT_COMMAND 0xDB

#define SSD1306_DISPLAY_ALL_ON_RESUME_COMMAND 0xA4

#define SSD1306_SET_START_LINE_COMMAND 0x40
#define SSD1306_EXTERNAL_VCC 0x01           // External display voltage source
#define SSD1306_SWITCH_CAP_VCC 0x02          // Gen. display voltage from 3.3V

#define SSD1306_RIGHT_HORIZONTAL_SCROLL_COMMAND 0x26              // Init rt scroll
#define SSD1306_LEFT_HORIZONTAL_SCROLL_COMMAND 0x27               // Init left scroll
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL_COMMAND 0x29 // Init diag scroll
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL_COMMAND 0x2A  // Init diag scroll
#define SSD1306_DEACTIVATE_SCROLL_COMMAND 0x2E                    // Stop scroll
#define SSD1306_ACTIVATE_SCROLL_COMMAND 0x2F                      // Start scroll
#define SSD1306_SET_VERTICAL_SCROLL_AREA_COMMAND 0xA3             // Set scroll range

// Device Vcc supply source. Choose from SSD1306_EXTERNAL_VCC or SSD1306_SWITCH_CAP_VCC
#define SSD1306_VCC_STATE SSD1306_SWITCH_CAP_VCC

typedef enum SSD1306PixelColor {
    SSD1306_BLACK,
    SSD1306_WHITE
} SSD1306PixelColor;


I2CStatus initSSD1306(I2C_TypeDef *I2Cx);
void clearDisplaySSD1306();
void invertDisplaySSD1306(bool isSetInverted);

void printCharSSD1306(char character, uint8_t x, uint8_t y);
void printStringSSD1306(uint8_t x, uint8_t y, char *string);
void printfStringSSD1306(uint8_t x, uint8_t y, char *format, ...);

void printBitmapSSD1306(uint8_t x, uint8_t y, const uint8_t *bitMapArray, uint8_t rows, uint8_t columns);
void drawPixelSSD1306(uint8_t x, uint8_t y, SSD1306PixelColor pixelColor);
void refreshDisplaySSD1306();

/*
 * Activate a right handed scroll for rows start through stop
 * The display is 16 rows tall. To scroll the whole display, run: startScrollRightSSD1306(0x00, 0x0F)
 */
void startScrollRightSSD1306(uint8_t startRow, uint8_t stopRow);

/*
 * Activate a left handed scroll for rows start through stop
 * The display is 16 rows tall. To scroll the whole display, run: startScrollLeftSSD1306(0x00, 0x0F)
 */
void startScrollLeftSSD1306(uint8_t startRow, uint8_t stopRow);

/*
 * Activate a diagonal scroll for rows start through stop
 * The display is 16 rows tall. To scroll the whole display, run: startDiagonalScrollRightSSD1306(0x00, 0x0F)
 */
void startDiagonalScrollRightSSD1306(uint8_t startRow, uint8_t stopRow);

/*
 * Activate a diagonal scroll for rows start through stop
 * The display is 16 rows tall. To scroll the whole display, run: startDiagonalScrollLeftSSD1306(0x00, 0x0F)
 */
void startDiagonalScrollLeftSSD1306(uint8_t startRow, uint8_t stopRow);

void stopScrollSSD1306();
void dimDisplaySSD1306(bool isDimmed);
void drawLineSSD1306(uint8_t startX, uint8_t endX, uint8_t startY, uint8_t endY);
void sendCommandSSD1306(uint8_t command);