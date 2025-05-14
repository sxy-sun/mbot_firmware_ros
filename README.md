# MBot Firmware (ROS2 Jazzy)
## Project Components

1. `libmicroros`: This directory contains the precompiled micro-ROS static library (`libmicroros.a`) and all necessary header files for the Raspberry Pi Pico. This library includes:
    - ROS 2 client library core functionality
    - Message type definitions
    - Transport layer implementations
    - Serialization/deserialization utilities
2. `microros_static_library`: Contains scripts and configuration files used to generate the `libmicroros` static library. We use it when we need to add customized ros data types. The key components include:
    - `library_generation.sh`: Script that sets up the build environment, compiles micro-ROS packages, and generates the static library
    - `colcon.meta`: Configuration for the colcon build system
    - `toolchain.cmake`: CMake toolchain file for cross-compiling to Raspberry Pi Pico

3. `pico_uart_transport.c` & `pico_uart_transports.h`: These files implement the UART-based transport layer for micro-ROS on the Pico. The transport layer is responsible for:
    - Opening and closing serial communication
    - Reading and writing data over UART
    - Managing timeouts and error handling
    - Providing POSIX-like timing functions (`usleep` and `clock_gettime`)

4. `mbot`: MBot Hardware Library
5. `rc` (Robot Control): A library providing essential functions for robot control applications.
6. Supporting Files
    - `available_ros2_types`: A list of all ROS 2 message, service, and action types available in the micro-ROS library
    - `built_packages`: A list of all Git repositories and their specific commit hashes used to build the micro-ROS library

## Getting Started

1. Connect your Raspberry Pi Pico to your computer
2. Build the project using CMake and the Pico SDK
3. Flash the generated UF2 file to your Pico
4. Run a micro-ROS agent on your host computer to bridge between the Pico and the ROS 2 network
5. Start publishing/subscribing to ROS 2 topics from your Pico
