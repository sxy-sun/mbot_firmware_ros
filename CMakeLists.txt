# Minimum required CMake version for this project
cmake_minimum_required(VERSION 3.12)

# Include the Pico SDK CMake configuration
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Define the project name and supported languages
project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK - must be called after project()
pico_sdk_init()

# Add the libmicroros directory to the linker search path
link_directories(libmicroros)

#=============================================================================
# Include the MBot library and Robot Control library
# Note: The RC library depends on the MBot library, so mbot must be added first
#=============================================================================
add_subdirectory(mbot)
add_subdirectory(rc)

# Define the executable target and its source files
add_executable(pico_micro_ros_example
    pico_micro_ros_example.c     # Main application code
    pico_uart_transport.c        # UART transport implementation
    # Add your additional source files here
)
# Link required libraries to the executable
target_link_libraries(pico_micro_ros_example
    pico_stdlib                  # The Pico standard library
    microros                     # The micro-ROS library
    mbotlib                      # MBot hardware library
    rclib                        # Robot Control library
    # Add your additional libraries here
)

# Add include directories for header files
target_include_directories(pico_micro_ros_example PUBLIC
    libmicroros/include          # micro-ROS headers
    include                      # Project-level include directory with configurations
    # Add your additional include directories here
)

#=============================================================================
# [DO NOT CHANGE] Optimization settings
# These settings help reduce binary size - important for microcontrollers
#=============================================================================
# -ffunction-sections and -fdata-sections allow unused code removal during linking
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

#=============================================================================
# [DO NOT CHANGE] Pico-specific communication settings
# These settings configure USB communication for micro-ROS
#=============================================================================
# Enable USB for stdio (for communication with host)
pico_enable_stdio_usb(pico_micro_ros_example 1)
# Disable default UART for stdio (using custom implementation instead)
pico_enable_stdio_uart(pico_micro_ros_example 0)
# Disable CRLF handling for better performance with micro-ROS
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
pico_add_extra_outputs(pico_micro_ros_example)
