cmake_minimum_required(VERSION 3.20)
project(SSD1306 C)

set(CMAKE_C_STANDARD 99)

include(cmake/CPM.cmake)

CPMAddPackage(
        NAME STM32Core
        GITHUB_REPOSITORY ximtech/STM32Core
        GIT_TAG origin/main)

set(SSD1306_DIRECTORY
        ${I2C_POLLING_DIRECTORY}
        ${SSD1306_SOURCE_DIR}
        ${SSD1306_SOURCE_DIR}/include
        CACHE STRING "SSD1306 directories include to the main project" FORCE)

set(SSD1306_SOURCES
        ${I2C_POLLING_SOURCES}
        ${SSD1306_SOURCE_DIR}/SSD1306.c
        ${SSD1306_SOURCE_DIR}/include/SSD1306.h
        CACHE STRING "SSD1306 source files include to the main project" FORCE)