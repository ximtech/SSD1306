# SSD1306
**STM32** Low Layer(LL) library. Monochrome OLEDs based on SSD1306 driver. Uses I2C

### SSD1306 128x64
<img src="https://github.com/ximtech/SSD1306/blob/main/example/SSD1306_128x64.PNG" alt="image" width="300"/>

### SSD1306 128x32
<img src="https://github.com/ximtech/SSD1306/blob/main/example/SSD1306_128x32.jpg" alt="image" width="300"/>

### SSD1306 96x16
<img src="https://github.com/ximtech/SSD1306/blob/main/example/SSD1306_96x16.PNG" alt="image" width="300"/>

### Features

- Popular display support
- Display scroll
- printf function support
- Draw function support


### Add as CPM project dependency

How to add CPM to the project, check the [link](https://github.com/cpm-cmake/CPM.cmake)

```cmake
CPMAddPackage(
        NAME SSD1306
        GITHUB_REPOSITORY ximtech/SSD1306
        GIT_TAG origin/main)
```

### Project configuration

1. Start project with STM32CubeMX:
    * [I2C configuration](https://github.com/ximtech/SSD1306/blob/main/example/config.PNG)
2. Select: Project Manager -> Advanced Settings -> I2C -> LL
3. Generate Code
4. Add sources to project:

```cmake
add_subdirectory(${STM32_CORE_SOURCE_DIR}/I2C/Polling)  # add I2C dependency

include_directories(${includes}
        ${SSD1306_DIRECTORY})   # matrix display source directories

file(GLOB_RECURSE SOURCES ${sources}
        ${SSD1306_SOURCES})    # matrix display source files
```

3. Then Build -> Clean -> Rebuild Project

## Wiring

- <img src="https://github.com/ximtech/SSD1306/blob/main/example/wiring.png" alt="image" width="300"/>

## Usage

In `SSD1306.h` default defines. Override them in `main.h` if needed

```c
// Display resolution
#define SSD1306_RESOLUTION_WIDTH 128
#define SSD1306_RESOLUTION_HEIGHT 32

#define SSD1306_I2C_TIMEOUT_MS 100
#define SD1306_DEVICE_ADDRESS 0x78 // Notice: SSD1306 128x32 can only have one I2C address

// Font define section. Uncomment/comment for font enable or disable
#define SSD1306_FONT_DEFAULT  1
```

- Usage example: [link](https://github.com/ximtech/SSD1306/blob/main/example/example.c)