# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Pico C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(Pico Pico.c )

pico_set_program_name(Pico "Pico")
pico_set_program_version(Pico "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Pico 0)
pico_enable_stdio_usb(Pico 1)

# Add the standard library to the build
target_link_libraries(Pico
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(Pico PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(Pico)


# ---------- Drivers library (every driver .c file goes in here) ----------
add_library(drivers STATIC
    drivers/led.c
    drivers/ultrasonic.c
    drivers/motor.c


)
target_include_directories(drivers PUBLIC ${CMAKE_CURRENT_LIST_DIR}/drivers)
target_link_libraries(drivers PUBLIC pico_stdlib hardware_pwm)

# ---------- Tests: one separate program per driver ----------
add_executable(test_led tests/test_led.c)
target_link_libraries(test_led PRIVATE drivers)
pico_enable_stdio_uart(test_led 0)   # UART0 (GP0/GP1) is reserved for the ESP32 link
pico_enable_stdio_usb(test_led 1)
pico_add_extra_outputs(test_led)
add_executable(test_ultrasonic tests/test_ultrasonic.c)
target_link_libraries(test_ultrasonic PRIVATE drivers)
pico_enable_stdio_uart(test_ultrasonic 0)
pico_enable_stdio_usb(test_ultrasonic 1)
pico_add_extra_outputs(test_ultrasonic)
add_executable(test_motor tests/test_motor.c)
target_link_libraries(test_motor PRIVATE drivers)
pico_enable_stdio_uart(test_motor 0)
pico_enable_stdio_usb(test_motor 1)
pico_add_extra_outputs(test_motor)