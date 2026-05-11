# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(PlanB C CXX ASM)

pico_sdk_init()
add_executable(PlanB PlanB.cpp)

pico_set_program_name(PlanB "PlanB")
pico_set_program_version(PlanB "0.1")


pico_enable_stdio_uart(PlanB 0)
pico_enable_stdio_usb(PlanB 1)  # changed to 1

target_link_libraries(PlanB
        pico_stdlib
        hardware_pwm
        pico_cyw43_arch_none)  # added

target_include_directories(PlanB PRIVATE
        ${CMAKE_CURRENT_LIST_DIR})

pico_add_extra_outputs(PlanB)