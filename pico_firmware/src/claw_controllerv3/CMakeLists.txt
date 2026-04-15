# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# # == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
# if(WIN32)
#     set(USERHOME $ENV{USERPROFILE})
# else()
#     set(USERHOME $ENV{HOME})
# endif()
# set(sdkVersion 2.2.0)
# set(toolchainVersion 14_2_Rel1)
# set(picotoolVersion 2.2.0-a4)
# set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
# if (EXISTS ${picoVscode})
#     include(${picoVscode})
# endif()
# # ====================================================================================

set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/pico-sdk)
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(claw_controllerv3 C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(claw_controllerv3 claw_controllerv3.cpp )

pico_set_program_name(claw_controllerv3 "claw_controllerv3")
pico_set_program_version(claw_controllerv3 "0.1")

add_executable(claw_dc claw_dc.cpp) #SABRINA ADDED 
pico_add_extra_outputs(claw_dc) #SABRINA ADDED

add_executable(drill drill.cpp) #SABRINA ADDED 
pico_add_extra_outputs(drill) #SABRINA ADDED

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(claw_controllerv3 0)
pico_enable_stdio_usb(claw_controllerv3 1)
pico_enable_stdio_usb(claw_dc 1)
pico_enable_stdio_uart(claw_dc 0) #SABRINA ADDED
pico_enable_stdio_usb(drill 1)
pico_enable_stdio_uart(drill 0) #SABRINA ADDED


# Add the standard library to the build
target_link_libraries(claw_controllerv3
        pico_stdlib)

target_link_libraries(claw_dc pico_stdlib hardware_pwm hardware_gpio) #SABRINA ADDED 

# Add the standard include files to the build
target_include_directories(claw_controllerv3 PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(claw_controllerv3 
            hardware_pwm
            hardware_clocks
            pico_cyw43_arch_none
        
        )
target_link_libraries(drill pico_stdlib hardware_pwm) #SABRINA ADDED

pico_add_extra_outputs(claw_controllerv3)

