cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(sdcard C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)

add_executable(sdcard
    main.c
    hw_config.c
)

add_compile_definitions(USE_PRINTF)

target_link_libraries(sdcard
    pico_stdlib
    hardware_i2c
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)

pico_enable_stdio_usb(sdcard 1)
pico_enable_stdio_uart(sdcard 1)

pico_add_extra_outputs(sdcard)
