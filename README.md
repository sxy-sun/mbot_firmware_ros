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


## MicroROS Integration Plan

### Overview
We are transitioning the MBot firmware from using LCM (Lightweight Communications and Marshalling) to using microROS for communication. This will allow the MBot to directly interact with ROS 2 ecosystems without requiring custom bridges or adaptations.

### Directory Structure
- `old_src/`: Original firmware using LCM communication
- `src/`: New microROS-based implementation (in progress)

### Integration Strategy
1. **Create ROS Communication Layer**
   - Implement a complete microROS interface in `mbot_classic_ros.c/h`
   - Map existing MBot data structures to standard ROS 2 message types
   - Maintain same hardware control functionality

2. **Message Type Mapping**
   - IMU data → `sensor_msgs/Imu`
   - Odometry → `nav_msgs/Odometry`
   - Robot velocity → `geometry_msgs/Twist`
   - Wheel state → `sensor_msgs/JointState`
   - Motor commands → `std_msgs/Float32MultiArray`
   - Encoder counts → `std_msgs/Int32MultiArray`
   - Analog inputs → `std_msgs/Float32MultiArray`

3. **Current Source Files**
   ```
   src/
   ├── mbot_classic_ros.h       - MicroROS interface declaration
   └── mbot_classic_ros.c       - MicroROS implementation + main entry point
   ```

4. **Implementation Steps**
   - [x] Basic microROS setup skeleton
   - [x] Configure CMakeLists.txt for the new firmware
   - [x] Define data structures for robot state tracking
   - [x] Set up main function and initialization structure
   - [x] Initialize executor framework with timer and subscribers
   - [ ] Next: Implement mbot_init_micro_ros_comm() to create publishers/subscribers
   - [ ] Implement timer_callback() for periodic publishing
   - [ ] Implement message conversion from internal state to ROS messages
   - [ ] Implement subscriber callbacks to handle incoming commands
   - [ ] Connect hardware interface to sensor reading
   - [ ] Test with ROS 2 ecosystem

### Key Technical Considerations
1. **No Custom Message Types**
   - Using only standard ROS 2 message types available in microROS
   - Avoiding custom message definitions to simplify integration

2. **Control Flow Differences**
   - Old: LCM callback-driven architecture
   - New: Timer-based publishing and executor-based subscription handling

3. **Timing Management**
   - Using ROS time synchronization instead of custom timestamps
   - Maintaining fixed-rate control loops (10Hz for state publishing)

4. **State Tracking**
   - Using local state variables for robot state
   - Synchronizing state between hardware readings and ROS messages

5. **CRLF Handling**
   - CRLF handling is disabled for performance optimization
   - All print statements use `\r\n` instead of just `\n`

### Testing Plan
1. Basic connectivity with ROS 2 system
2. Verify all topics publish correctly
3. Command robot through ROS 2 topics
4. Validate equivalent functionality to LCM version

### Next Steps
1. Implement the `mbot_init_micro_ros_comm()` function to initialize publishers, subscribers, and services
2. Add message initialization and conversion code
3. Implement callback functions for handling ROS messages
4. Connect to hardware interfaces for sensor reading and motor control
