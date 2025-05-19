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

#=============================================================================
# Global include directories for all targets
#=============================================================================
include_directories(
    libmicroros/include          # micro-ROS headers
    include                      # Project-level include directory with configurations
    ${PICO_SDK_PATH}/lib/tinyusb/src
)

#=============================================================================
# Include the MBot library and Robot Control library
# Note: The RC library depends on the MBot library, so mbot must be added first
#=============================================================================
add_subdirectory(mbot)
add_subdirectory(rc)

# Define the main executable target for MBot Classic ROS firmware
add_executable(mbot_classic_ros
    src/mbot_classic_ros.c       # Main application code
    src/mbot_classic_ros.h       # Header file
    src/mbot_odometry.c          # Odometry calculation
    src/mbot_odometry.h          # Odometry header
    src/usb_descriptors.c        # USB descriptors
    src/dual_cdc.c               # Custom dual CDC implementation
    include/pico_uart_transport.c  # UART transport implementation
)

# Link required libraries to the executable
target_link_libraries(mbot_classic_ros
    pico_stdlib                  # The Pico standard library
    pico_multicore               # For multicore support
    hardware_i2c                 # I2C hardware support
    hardware_pio                 # PIO hardware support
    hardware_pwm                 # PWM hardware support
    hardware_adc                 # ADC hardware support
    hardware_sync                # Hardware synchronization
    microros                     # The micro-ROS library
    mbotlib                      # MBot hardware library
    rclib                        # Robot Control library
    tinyusb_device               # TinyUSB device support
    tinyusb_board                # TinyUSB board support
)

# [DO NOT CHANGE]
# Disable CRLF handling for better performance with micro-ROS for main application
# This means when printing to the console you need to use \r\n instead of \n
target_compile_definitions(mbot_classic_ros PRIVATE 
    PICO_UART_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_ENABLE_CRLF_SUPPORT=0
    PICO_STDIO_DEFAULT_CRLF=0
)

#=============================================================================
# Test programs - streamlined for Classic MBot only
#=============================================================================
# Motor Test for Classic MBot
add_executable(mbot_classic_motor_test
    tests/mbot_classic_motor_test.c
)
target_link_libraries(mbot_classic_motor_test
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_pwm
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
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_pwm
    mbotlib
    rclib
)
pico_enable_stdio_usb(mbot_encoder_test 1)
pico_enable_stdio_uart(mbot_encoder_test 0)
pico_add_extra_outputs(mbot_encoder_test)

# Motor Calibration for Classic MBot
add_executable(mbot_calibrate_classic
    tests/mbot_calibrate_classic.c
)
target_link_libraries(mbot_calibrate_classic
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_pwm
    mbotlib
    rclib
)
pico_enable_stdio_usb(mbot_calibrate_classic 1)
pico_enable_stdio_uart(mbot_calibrate_classic 0)
pico_add_extra_outputs(mbot_calibrate_classic)

# Analog Input Test
add_executable(mbot_analog_input_test
    tests/mbot_analog_input_test.c
)
target_link_libraries(mbot_analog_input_test
    pico_stdlib
    hardware_i2c
    hardware_pio
    hardware_pwm
    hardware_adc
    mbotlib
    rclib
)
pico_enable_stdio_usb(mbot_analog_input_test 1)
pico_enable_stdio_uart(mbot_analog_input_test 0)
pico_add_extra_outputs(mbot_analog_input_test)

# Dual CDC Test
add_executable(dual_cdc_test
    tests/dual_cdc_test.c
    src/dual_cdc.c
    src/usb_descriptors.c
)
target_include_directories(dual_cdc_test PRIVATE include)
target_link_libraries(dual_cdc_test
    pico_stdlib
    hardware_clocks
    tinyusb_device
    tinyusb_board
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
# -ffunction-sections and -fdata-sections allow unused code removal during linking
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

#=============================================================================
# [DO NOT CHANGE] Pico-specific communication settings
#=============================================================================
# Enable USB for stdio (for communication with host)
pico_enable_stdio_usb(mbot_classic_ros 1)
# Disable default UART for stdio (using custom implementation instead)
pico_enable_stdio_uart(mbot_classic_ros 0)
# Enable two CDC interfaces for dual USB serial ports
target_compile_definitions(mbot_classic_ros PRIVATE CFG_TUD_CDC=2)

# Generate UF2
pico_add_extra_outputs(mbot_classic_ros)