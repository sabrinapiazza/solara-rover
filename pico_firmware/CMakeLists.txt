cmake_minimum_required(VERSION 3.13)

set(PICO_SDK_PATH $ENV{HOME}/pico-sdk)
# set(PICO_BOARD pico_w)
set(PICO_BOARD pico)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(solara_firmware C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(solara_firmware
    src/main.cpp
    src/motor_driver.cpp
    src/encoder.cpp
)

pico_enable_stdio_usb(solara_firmware 1)
pico_enable_stdio_uart(solara_firmware 0)

target_link_libraries(solara_firmware
    pico_stdlib
    # pico_cyw43_arch_none
    hardware_pwm
    hardware_gpio
)

target_include_directories(solara_firmware PRIVATE src)
pico_add_extra_outputs(solara_firmware)

target_compile_definitions(solara_firmware PRIVATE
    PICO_STDIO_USB_CONNECTION_WITHOUT_DTR=1
)