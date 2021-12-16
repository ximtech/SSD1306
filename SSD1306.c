#include "SSD1306.h"

#define GDDRAM_PAGES_COUNT   (SSD1306_RESOLUTION_HEIGHT / 8)
#define GDDRAM_SEGMENTS_COUNT SSD1306_RESOLUTION_WIDTH

#define SINGLE_COMMAND_CONTROL_BYTE 0x00
#define MULTIPLE_COMMANDS_CONTROL_BYTE 0x80

#define SINGLE_DATA_BYTE_CONTROL_BYTE 0x40
#define MULTIPLE_DATA_BYTES_CONTROL_BYTE 0xC0

#if SSD1306_RESOLUTION_WIDTH == 128 && SSD1306_RESOLUTION_HEIGHT == 32
    #define SSD1306_COM_PINS_VALUE 0x02
    #define SSD1306_CONTRAST_VALUE 0x8F

#elif SSD1306_RESOLUTION_WIDTH == 128 && SSD1306_RESOLUTION_HEIGHT == 64
    #define SSD1306_COM_PINS_VALUE 0x12
    #if SSD1306_VCC_STATE == SSD1306_EXTERNAL_VCC
        #define SSD1306_CONTRAST_VALUE 0xCF
    #else
        #define SSD1306_CONTRAST_VALUE 0x9F
    #endif

#elif SSD1306_RESOLUTION_WIDTH == 96 && SSD1306_RESOLUTION_HEIGHT == 16
    #define SSD1306_COM_PINS_VALUE 0x20
    #if SSD1306_VCC_STATE == SSD1306_EXTERNAL_VCC
        #define SSD1306_CONTRAST_VALUE 0x10
    #else
        #define SSD1306_CONTRAST_VALUE 0xAF
    #endif

#else
    #error "Invalid SSD1306 Resolution values! Available displays: 128x64, 128x32, 96x16"
#endif

#define SSD1306_DISPLAY_CLOCK_DIVISION_VALUE 0x80   // the suggested ratio 0x80
#define SSD1306_NO_OFFSET_VALUE 0x00   // the suggested ratio 0x80
#define SSD1306_HORIZONTAL_ADDRESSING_MODE_VALUE 0x00  // After the display RAM is read/written, the column address pointer is increased auto by 1
#define SSD1306_ENABLE_CHARGE_PUMP_VALUE 0x14  // must be enabled by the following command: 8Dh
#define SSD1306_COLUMN_START_ADDRESS_VALUE 0x00 // Column start address (0 = reset)
#define SSD1306_PAGE_START_ADDRESS_VALUE 0x00   // Page start address (0 = reset)

#if SSD1306_VCC_STATE == SSD1306_EXTERNAL_VCC
    #define SSD1306_PRE_CHARGE_VALUE 0x22  // The interval is counted in number of DCLK, where RESET equals 2 DCLKs
#else
    #define SSD1306_PRE_CHARGE_VALUE 0xF1  // The interval is counted in number of DCLK, where RESET equals 2 DCLKs
#endif

#define ASCII_SPACE_OFFSET 0x20
#define CHAR_WIDTH_PIXELS  6
#define CHAR_HEIGHT_PIXELS 8

#define SSD1306_FONT_MAX_CHARACTER_CAPACITY ((SSD1306_RESOLUTION_WIDTH / CHAR_WIDTH_PIXELS) * (SSD1306_RESOLUTION_HEIGHT / CHAR_HEIGHT_PIXELS))

#define ABS(N) ((N < 0) ? (-N) : (N))
#define SWAP(A, B) { int16_t temp = A; A = B; B = temp; }
#define BIT_READ(value, bit) (((value) >> (bit)) & 0x01)

static I2C_Polling i2c;
static uint8_t deviceGDDRAM[GDDRAM_PAGES_COUNT][GDDRAM_SEGMENTS_COUNT];

static I2CStatus sendCommands(uint32_t commandCount, ...);
static inline void clearDeviceGDDRAM();


I2CStatus initSSD1306(I2C_TypeDef *I2Cx) {
    i2c = initI2C(I2Cx, I2C_ADDRESSING_MODE_7BIT, SSD1306_I2C_TIMEOUT_MS);
    I2CStatus status = isDeviceReady(&i2c, SD1306_DEVICE_ADDRESS);
    if (status != I2C_OK) return status;

    sendCommands(26,
                 SSD1306_DISPLAY_OFF_COMMAND,
                 SSD1306_SET_DISPLAY_CLOCK_DIVISION_COMMAND,
                 SSD1306_DISPLAY_CLOCK_DIVISION_VALUE,
                 SSD1306_SET_MULTIPLEX_COMMAND,
                 SSD1306_RESOLUTION_HEIGHT - 1,
                 SSD1306_SET_DISPLAY_OFFSET_COMMAND,
                 SSD1306_NO_OFFSET_VALUE,
                 SSD1306_SET_START_LINE_COMMAND,
                 SSD1306_CHARGE_PUMP_COMMAND,
                 SSD1306_ENABLE_CHARGE_PUMP_VALUE,
                 SSD1306_SET_MEMORY_MODE_COMMAND,
                 SSD1306_HORIZONTAL_ADDRESSING_MODE_VALUE,
                 SSD1306_SEGRE_MAP_COMMAND | 0x1,
                 SSD1306_COMMUNICATION_SCAN_DETECTED_COMMAND,
                 SSD1306_SET_COM_PINS_COMMAND,
                 SSD1306_COM_PINS_VALUE,
                 SSD1306_SET_CONTRAST_COMMAND,
                 SSD1306_CONTRAST_VALUE,
                 SSD1306_SET_PRE_CHARGE_COMMAND,
                 SSD1306_PRE_CHARGE_VALUE,
                 SSD1306_SET_V_COM_DETECT_COMMAND,
                 SSD1306_SET_START_LINE_COMMAND,
                 SSD1306_DISPLAY_ALL_ON_RESUME_COMMAND,
                 SSD1306_NORMAL_DISPLAY_COMMAND,
                 SSD1306_DEACTIVATE_SCROLL_COMMAND,
                 SSD1306_DISPLAY_ON_COMMAND);

    clearDeviceGDDRAM();    // init array
    return status;
}

void printCharSSD1306(char character, uint8_t x, uint8_t y) {
    #ifdef SSD1306_FONT_DEFAULT
    uint8_t charIndex = character - ASCII_SPACE_OFFSET;
    for (uint8_t i = 0; i < CHAR_WIDTH_PIXELS; i++) {
        for (uint8_t j = 0; j < CHAR_HEIGHT_PIXELS; j++) {
            uint8_t pixelValue = BIT_READ(SSD1306_ASCII_DEFAULT_128X32[charIndex][i], j);
            drawPixelSSD1306(x + i, y + j, pixelValue);
        }
    }
    #endif
}

void printStringSSD1306(uint8_t x, uint8_t y, char *string) {
    for (int i = 0; i < strlen(string); i++) {
        printCharSSD1306(string[i], x, y);
        x += CHAR_WIDTH_PIXELS;
    }
    refreshDisplaySSD1306();
}

void printfStringSSD1306(uint8_t x, uint8_t y, char *format, ...) {
    static char formatBuffer[SSD1306_FONT_MAX_CHARACTER_CAPACITY];
    memset(formatBuffer, 0, SSD1306_FONT_MAX_CHARACTER_CAPACITY);
    va_list args;

    va_start(args, format);
    vsnprintf(formatBuffer, SSD1306_FONT_MAX_CHARACTER_CAPACITY, format, args);
    va_end(args);
    printStringSSD1306(x, y, formatBuffer);
}

void printBitmapSSD1306(uint8_t x, uint8_t y, const uint8_t *bitMapArray, uint8_t rows, uint8_t columns) {
    for (uint8_t i = 0; i < rows; i++) {
        for (uint8_t j = 0; j < columns; j++) {
            for (uint8_t k = 0; k < 8; k++) {
                drawPixelSSD1306(x + 8 * j + k, y + i, bitMapArray[columns * i + j] & (0x80 >> k));
            }
        }
    }
    refreshDisplaySSD1306();
}

void drawPixelSSD1306(uint8_t x, uint8_t y, SSD1306PixelColor pixelColor) {
    if (x < SSD1306_RESOLUTION_WIDTH && y < SSD1306_RESOLUTION_HEIGHT) {
        uint8_t pixel = 0x01;
        uint8_t lineNumber = y >> (GDDRAM_PAGES_COUNT - 1);
        uint8_t byte = pixel << (y % 8);

        uint8_t pixelValue = deviceGDDRAM[lineNumber][x];
        deviceGDDRAM[lineNumber][x] = (pixelColor == SSD1306_WHITE) ? (pixelValue | byte) : (pixelValue & ~byte);
    }
}

void refreshDisplaySSD1306() {
    sendCommands(6,
                 SSD1306_SET_COLUMN_ADDRESS_COMMAND,
                 SSD1306_COLUMN_START_ADDRESS_VALUE,
                 (SSD1306_RESOLUTION_WIDTH - 1),   // Display width
                 SSD1306_SET_PAGE_ADDRESS_COMMAND,
                 SSD1306_PAGE_START_ADDRESS_VALUE,
                 (GDDRAM_PAGES_COUNT - 1));   // Page end address

    if (startAsMasterI2C(&i2c, SD1306_DEVICE_ADDRESS, I2C_WRITE_TO_SLAVE) == I2C_OK) {
        transmitByteAsMasterI2C(&i2c, SINGLE_DATA_BYTE_CONTROL_BYTE);
        for (uint16_t line = 0; line < GDDRAM_PAGES_COUNT; line++) {
            for (uint8_t column = 0; column < SSD1306_RESOLUTION_WIDTH; column++) {
                transmitByteAsMasterI2C(&i2c, deviceGDDRAM[line][column]);
            }
        }
        stopAsMasterI2C(&i2c);
    }
}

void clearDisplaySSD1306() {
    clearDeviceGDDRAM();
    refreshDisplaySSD1306();
}

void invertDisplaySSD1306(bool isSetInverted) {
    if (isSetInverted) {
        sendCommandSSD1306(SSD1306_INVERT_DISPLAY_COMMAND);
    } else {
        sendCommandSSD1306(SSD1306_NORMAL_DISPLAY_COMMAND);
    }
}

void startScrollRightSSD1306(uint8_t startRow, uint8_t stopRow) {
    sendCommands(8, SSD1306_RIGHT_HORIZONTAL_SCROLL_COMMAND,
                 0x00, startRow, 0x00, stopRow, 0x00, 0xFF,
                 SSD1306_ACTIVATE_SCROLL_COMMAND);
}

void startScrollLeftSSD1306(uint8_t startRow, uint8_t stopRow) {
    sendCommands(8, SSD1306_LEFT_HORIZONTAL_SCROLL_COMMAND,
                 0x00, startRow, 0x00, stopRow, 0x00, 0xFF,
                 SSD1306_ACTIVATE_SCROLL_COMMAND);
}

void startDiagonalScrollRightSSD1306(uint8_t startRow, uint8_t stopRow) {
    sendCommands(10, SSD1306_SET_VERTICAL_SCROLL_AREA_COMMAND,
                 0x00, SSD1306_RESOLUTION_HEIGHT,
                 SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL_COMMAND,
                 0x00, startRow, 0x00, stopRow, 0x01,
                 SSD1306_ACTIVATE_SCROLL_COMMAND);
}

void startDiagonalScrollLeftSSD1306(uint8_t startRow, uint8_t stopRow) {
    sendCommands(10, SSD1306_SET_VERTICAL_SCROLL_AREA_COMMAND,
                 0x00, SSD1306_RESOLUTION_HEIGHT,
                 SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL_COMMAND,
                 0x00, startRow, 0x00, stopRow, 0x01,
                 SSD1306_ACTIVATE_SCROLL_COMMAND);
}

void dimDisplaySSD1306(bool isDimmed) {
    uint8_t contrast = isDimmed ? 0 : 0xCF;
    sendCommands(2, SSD1306_SET_CONTRAST_COMMAND, contrast);
}

void stopScrollSSD1306() {
    sendCommandSSD1306(SSD1306_DEACTIVATE_SCROLL_COMMAND);
}

void drawLineSSD1306(uint8_t startX, uint8_t endX, uint8_t startY, uint8_t endY) {
    int16_t lineSteepness = ABS(endY - startY) > ABS(endX - startX);

    if (lineSteepness > 0) {
        SWAP(startX, endX)
        SWAP(endX, endY)
    }

    if (startX > endX) {
        SWAP(startX, endX)
        SWAP(startY, endY)
    }

    int32_t deltaX = endX - startX;
    int16_t deltaY = ABS(endY - startY);

    int32_t error = deltaX / 2;
    int16_t steepnessOfY = startY < endY ? 1 : -1;

    while (startX <= endX) {
        if (lineSteepness > 0) {
            drawPixelSSD1306(startY, startX, SSD1306_WHITE);
        } else {
            drawPixelSSD1306(startX, startY, SSD1306_WHITE);
        }

        error -= deltaY;
        if (error < 0) {
            startY += steepnessOfY;
            error += deltaX;
        }
        startX++;
    }
    refreshDisplaySSD1306();
}

void sendCommandSSD1306(uint8_t command) {
    sendCommands(1, command);
}

static inline void clearDeviceGDDRAM() {
    memset(deviceGDDRAM, 0, sizeof(uint8_t) * GDDRAM_PAGES_COUNT * GDDRAM_SEGMENTS_COUNT);
}

static I2CStatus sendCommands(uint32_t commandCount, ...) {
    I2CStatus status = startAsMasterI2C(&i2c, SD1306_DEVICE_ADDRESS, I2C_WRITE_TO_SLAVE);
    if (status == I2C_OK) {
        va_list vaList;
        va_start(vaList, commandCount);    // initialize vaList for command number of arguments

        transmitByteAsMasterI2C(&i2c, SINGLE_COMMAND_CONTROL_BYTE);
        for (uint32_t i = 0; i < commandCount; i++) {
            uint8_t command = va_arg(vaList, uint32_t); // must be promoted to int
            transmitByteAsMasterI2C(&i2c, command);
        }
        stopAsMasterI2C(&i2c);
    }
    return status;
}