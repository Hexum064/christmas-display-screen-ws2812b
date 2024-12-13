# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
#set(PICO_SDK_PATH "/home/carlk/pi/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(gif_screen_driver C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

add_subdirectory(../FatFs_SPI build)

# Add executable. Default name is the project name, version 0.1
add_executable(gif_screen_driver 
    gif_screen_driver.c
    hw_config.c
    gifdec.c
    gifdec.h
    ssd1306.c
    ssd1306.h
    font.h
)

# Add the standard library and FatFS/SPI to the build
target_link_libraries(gif_screen_driver 
    pico_stdlib
    pico_multicore
    hardware_i2c
    FatFs_SPI
)

pico_set_program_name(gif_screen_driver "gif_screen_driver")
pico_set_program_version(gif_screen_driver "0.1")

# Choose source and destination for standard input and output:
#   See 4.1. Serial input and output on Raspberry Pi Pico in Getting started with Raspberry Pi Pico (https://datasheets.raspberrypi.org/pico/getting-started-with-pico.pdf)
#   and 2.7.1. Standard Input/Output (stdio) Support in Raspberry Pi Pico C/C++ SDK (https://datasheets.raspberrypi.org/pico/raspberry-pi-pico-c-sdk.pdf):
pico_enable_stdio_uart(gif_screen_driver 0)
pico_enable_stdio_usb(gif_screen_driver 1)

pico_add_extra_outputs(gif_screen_driver)

