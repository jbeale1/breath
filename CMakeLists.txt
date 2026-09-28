cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(fcounter C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_subdirectory(no-OS-FatFS-SD-SDIO-SPI-RPi-Pico/src build)

add_executable(fcounter
    main.c
    hw_config.c
)

pico_generate_pio_header(fcounter ${CMAKE_CURRENT_LIST_DIR}/freq_counter.pio)
pico_generate_pio_header(fcounter ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)


add_compile_definitions(USE_PRINTF)

target_link_libraries(fcounter
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_irq
    no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
)

pico_enable_stdio_usb(fcounter 1)
pico_enable_stdio_uart(fcounter 1)

pico_add_extra_outputs(fcounter)
