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
3. `comms`: Communication Setup
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

### Key Technical Considerations
1. **No Custom Message Types**
   - Using only standard ROS 2 message types available in microROS
   - Avoiding custom message definitions to simplify integration

2. **Control Flow Differences**
   - Old: LCM callback-driven architecture
   - New: Timer-based publishing and executor-based subscription handling

3. **Timing Management**
   - Using ROS time synchronization instead of custom timestamps
   - Maintaining fixed-rate control loops (25Hz for state publishing)

4. **State Tracking**
   - Using local state variables for robot state
   - Synchronizing state between hardware readings and ROS messages

5. **CRLF Handling**
   - CRLF handling is disabled for performance optimization
   - All print statements use `\r\n` instead of just `\n`

## Communication
The firmware uses a single USB Type-C connection with dual CDC (Communication Device Class) interfaces:
1. **Debug Channel** (`/dev/ttyACM0`):
   - Used for firmware debug messages and status prints
   - Accessible via standard serial tools: `sudo minicom -D /dev/ttyACM0 -b 115200`

2. **MicroROS Channel** (`/dev/ttyACM1`):
   - Dedicated to microROS communication
   - Handles all ROS2 messages and services
   - Used by micro-ros-agent for ROS2 bridge

This dual-channel approach allows simultaneous debugging and ROS communication without additional hardware connections.

### Important: Servicing TinyUSB and Using `sleep_ms`

**TinyUSB (the USB stack used for CDC communication) requires frequent servicing via `tud_task()` (`dual_cdc_task()`).**
If the USB stack is not serviced regularly (ideally every 1–10 ms), the host computer may think the device is unresponsive and disconnect the serial ports (`/dev/ttyACM0`, `/dev/ttyACM1`).

**Do NOT use long `sleep_ms()` calls.**

#### **Incorrect Usage (will cause USB ports to disappear):**
```c
// BAD: This will block USB for 2 seconds!
sleep_ms(2000);
```

#### **Correct Usage (keeps USB alive):**
Use the wait function we provide.
```c
#include "comms/dual_cdc.h";
mbot_wait_ms(2000);
```