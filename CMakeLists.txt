# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(fluid-simulation C CXX ASM)

pico_sdk_init()

file(GLOB INCLUDE "include/*.c")
file(GLOB SRC "src/*.c")

add_executable(fluid-simulation src/main.c ${INCLUDE})

pico_set_program_name(fluid-simulation "fluid-simulation")
pico_set_program_version(fluid-simulation "0.1")

pico_enable_stdio_uart(fluid-simulation 1)
pico_enable_stdio_usb(fluid-simulation 1)

target_link_libraries(fluid-simulation
        pico_stdlib)

target_include_directories(fluid-simulation PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(fluid-simulation 
        hardware_clocks
        hardware_gpio
        hardware_pwm
        hardware_i2c
        hardware_adc
        )
pico_add_extra_outputs(fluid-simulation)