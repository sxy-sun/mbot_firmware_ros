# Minimum required CMake version for this project
cmake_minimum_required(VERSION 3.12)

# Include the Pico SDK CMake configuration
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Define the project name and supported languages
project(mbot_firmware_ros C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK - must be called after project()
pico_sdk_init()

# Add the libmicroros directory to the linker search path
link_directories(libmicroros)

include_directories(
    libmicroros/include
    ${CMAKE_SOURCE_DIR}/include
    ${PICO_SDK_PATH}/lib/tinyusb/src
)

# Add subdirectories
add_subdirectory(mbot)
add_subdirectory(rc)
add_subdirectory(comms)

# Common libraries used by multiple targets
set(COMMON_LIBS
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_pwm
    hardware_adc
)

#=============================================================================
# Main MBot Classic ROS Executable
#=============================================================================
add_executable(mbot_classic_ros
    src/mbot_classic_ros.c
    src/mbot_classic_ros.h
    src/mbot_odometry.c
    src/mbot_odometry.h
)

target_link_libraries(mbot_classic_ros
    ${COMMON_LIBS}
    pico_multicore
    microros
    mbotlib
    rclib
    comms
)

# Configure USB and output settings for main executable
pico_enable_stdio_usb(mbot_classic_ros 1)
pico_enable_stdio_uart(mbot_classic_ros 0)
pico_add_extra_outputs(mbot_classic_ros)

# [DO NOT CHANGE]
# Disable CRLF handling for better performance with micro-ROS
# This means when printing to the console you need to use \r\n instead of \n
target_compile_definitions(mbot_classic_ros PRIVATE 
    # PICO_UART_ENABLE_CRLF_SUPPORT=0
    # PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    # PICO_STDIO_DEFAULT_CRLF=0
    CFG_TUD_CDC=2
)

#=============================================================================
# Test Programs
#=============================================================================

# MBot Calibration
add_executable(mbot_calibrate_classic
    tests/mbot_calibrate_classic.c
)
target_link_libraries(mbot_calibrate_classic
    ${COMMON_LIBS}
    mbotlib
    rclib
)
pico_enable_stdio_usb(mbot_calibrate_classic 1)
pico_enable_stdio_uart(mbot_calibrate_classic 0)
pico_add_extra_outputs(mbot_calibrate_classic)

# Motor Test
add_executable(mbot_classic_motor_test
    tests/mbot_classic_motor_test.c
)
target_link_libraries(mbot_classic_motor_test
    ${COMMON_LIBS}
    mbotlib
    rclib
)
pico_enable_stdio_usb(mbot_classic_motor_test 1)
pico_enable_stdio_uart(mbot_classic_motor_test 0)
pico_add_extra_outputs(mbot_classic_motor_test)

# Encoder Test
add_executable(mbot_encoder_test
    tests/mbot_encoder_test.c
)
target_link_libraries(mbot_encoder_test
    ${COMMON_LIBS}
    mbotlib
    rclib
)
pico_enable_stdio_usb(mbot_encoder_test 1)
pico_enable_stdio_uart(mbot_encoder_test 0)
pico_add_extra_outputs(mbot_encoder_test)

# Analog Input Test
add_executable(mbot_analog_input_test
    tests/mbot_analog_input_test.c
)
target_link_libraries(mbot_analog_input_test
    ${COMMON_LIBS}
    mbotlib
    rclib
)
pico_enable_stdio_usb(mbot_analog_input_test 1)
pico_enable_stdio_uart(mbot_analog_input_test 0)
pico_add_extra_outputs(mbot_analog_input_test)

# Dual CDC Test
add_executable(dual_cdc_test
    tests/dual_cdc_test.c
)
target_include_directories(dual_cdc_test PRIVATE
    include
    comms/include
)
target_link_libraries(dual_cdc_test
    pico_stdlib
    hardware_clocks
    comms
)
pico_add_extra_outputs(dual_cdc_test)

# LED Blink Test
add_executable(mbot_led_test
    tests/mbot_led_test.c
)
target_link_libraries(mbot_led_test 
    pico_stdlib
)
pico_enable_stdio_usb(mbot_led_test 1)
pico_enable_stdio_uart(mbot_led_test 0)
pico_add_extra_outputs(mbot_led_test)

#=============================================================================
# [DO NOT CHANGE] Optimization settings
# These settings help reduce binary size - important for microcontrollers
#=============================================================================
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")
