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
- `include/config/mbot_classic_config.h`: MBot classic config file
- `mbot/include/mbot/defs/mbot_params.h`: MBot system config file

### Code Modules in `src/`
- `mbot_classic_ros.c/.h`: Core application logic, main loop, hardware interface calls, and microROS node initialization.
- `mbot_ros_comms.c/.h`: Handles all ROS-specific communication aspects including publisher/subscriber/service setup, message initialization, and callbacks.
- `mbot_odometry.c/.h`: Odometry calculation utilities.
- `mbot_print.c/.h`: Debug printing utilities.

### Future plan
The function rclc_support_init in micro-ROS does require a connection to the micro-ROS Agent to succeed. When you call rclc_support_init, it attempts to establish communication with the Agent, and if the Agent is not reachable, the function will fail after a configurable timeout period. This means that initializing micro-ROS support with this function is not just a local setup; it actively tries to connect to the Agent as part of its process.

If the Agent is not running or cannot be reached, rclc_support_init will block and retry several times (by default, up to 10 times with a 1-second timeout per attempt), and then it will return an error if the connection cannot be established. This behavior can cause your application to "freeze" or hang during initialization if the Agent is not available.

The rmw_uros_ping_agent function allows you to verify if the micro-ROS Agent is available before calling initialization functions like rclc_support_init. This can help you avoid long blocking calls or failed initializations if the Agent is not running or reachable.

After initialization, if the connection to the Agent drops, micro-ROS does not automatically halt or crash. Instead, the client will continue running, but communication-dependent operations (like publishing or subscribing) will fail until the Agent is available again. For example, publishing will not work, and you may see error messages or failed status codes in your application. Timers and some other local logic can still execute, but their timing and behavior may be affected if they depend on Agent communication.

To handle reconnections robustly, you can implement a state machine in your application that:

Pings the Agent until it is reachable.

Initializes micro-ROS and creates entities when the Agent is available.

Monitors the connection and, if the Agent becomes unreachable, destroys entities and returns to waiting for the Agent to reconnect.

This approach allows your application to actively wait for reconnection and recover from Agent disconnections, rather than halting or requiring a restart


```
// States
enum State {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
};

// Global state tracker
State current_state = WAITING_AGENT;

void main() {
  while(true) {
    switch(current_state) {
      case WAITING_AGENT:
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) { // [1]
          current_state = AGENT_AVAILABLE;
        }
        break;

      case AGENT_AVAILABLE:
        if (initialize_micro_ros_entities()) { // Creates nodes, executors, etc [1]
          current_state = AGENT_CONNECTED;
        } else {
          destroy_entities(); // Cleanup if initialization fails [4]
          current_state = WAITING_AGENT;
        }
        break;

      case AGENT_CONNECTED:
        if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) { // [1]
          current_state = AGENT_DISCONNECTED;
        } else {
          rclc_executor_spin_some(executor, 100); // Normal operation
        }
        break;

      case AGENT_DISCONNECTED:
        destroy_entities(); // Critical cleanup step [4]
        current_state = WAITING_AGENT;
        break;
    }
  }
}

// Placeholder functions
bool initialize_micro_ros_entities() {
  // 1. rclc_support_init()
  // 2. Create nodes, publishers, subscribers
  // 3. Set up executors
  return true; // Return success status
}

void destroy_entities() {
  // 1. Destroy executors
  // 2. Destroy nodes
  // 3. Cleanup support
}
```

### Key Technical Considerations
1. **No Custom Message Types**
   - Using only standard ROS 2 message types available in microROS
   - Avoiding custom message definitions to simplify integration

2. **Control Flow Differences**
   - Old: LCM callback-driven architecture
   - New: Timer-based publishing and executor-based subscription handling

3. **Timing Management**
   - Using ROS time synchronization instead of custom timestamps
   - Maintaining fixed-rate control loops (e.g., 25Hz for sensor acquisition and control logic via `mbot_loop`)
   - Periodic ROS state publishing (e.g., 20Hz via `timer_callback` triggered by `ros_publish_timer`)

4. **State Tracking**
   - Using local state variables for robot state (`mbot_state_t`, `mbot_cmd_t`)
   - Synchronizing state between hardware readings and ROS messages (hardware readings update `mbot_state`, ROS messages are populated from `mbot_state`, `mbot_cmd` updated by ROS callbacks).
   Key global state variables:
     - `mbot_state_t mbot_state`: Defined in `mbot_classic_ros.c`, holds the current snapshot of all robot sensor data, odometry, and derived states. Updated by `mbot_loop`. Read by `mbot_publish_state`.
     - `mbot_cmd_t mbot_cmd`: Defined in `mbot_classic_ros.c`, stores the latest commands received via ROS subscriptions. Updated by ROS callbacks in `mbot_ros_comms.c`. Read by motor control logic in `mbot_loop`.
     - `mbot_params_t params`: Stores calibration parameters loaded from FRAM.

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

## Time 
- `mbot_state.timestamp_us` is the last time the mbot_state was updated by `mbot_loop`.
- `mbot_state.last_encoder_time` is the Pico local time recorded at the start of the encoder read in `mbot_loop`, used for calculating `encoder_delta_t`.
- `now` variable (local to `mbot_publish_state` function) holds the ROS-synchronized epoch time obtained via `rmw_uros_epoch_nanos()`, used for timestamping outgoing ROS messages.