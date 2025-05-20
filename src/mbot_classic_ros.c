/**
 * @file mbot_classic_ros.c
 * @brief MicroROS integration for MBot Classic
 */
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_srvs/srv/trigger.h>
#include <rmw_microros/time_sync.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/multicore.h>
#include <hardware/clocks.h>
#include <hardware/adc.h>

// mbotlib
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <mbot/fram/fram.h>
#include <mbot/imu/imu.h>

// mbot_classic_ros
#include "mbot_classic_ros.h"
#include "mbot_odometry.h"
#include "config/mbot_classic_config.h"

// comms
#include <comms/pico_uart_transports.h>
#include <comms/dual_cdc.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// Global state variables
mbot_state_t mbot_state = {0};
mbot_cmd_t mbot_cmd = {0};
mbot_params_t params;
mbot_bhy_config_t mbot_imu_config;
mbot_bhy_data_t mbot_imu_data;

// Global MicroROS objects
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

// Timer for periodic publishing
static rcl_timer_t timer;

// Define ROS publishers
static rcl_publisher_t imu_publisher;
static rcl_publisher_t odom_publisher;
static rcl_publisher_t encoders_publisher;
static rcl_publisher_t mbot_vel_publisher;
static rcl_publisher_t motor_vel_publisher;
static rcl_publisher_t motor_pwm_publisher;
static rcl_publisher_t analog_publisher;

// Define ROS messages
static sensor_msgs__msg__Imu imu_msg;
static nav_msgs__msg__Odometry odom_msg;
static std_msgs__msg__Int32MultiArray encoders_msg;
static geometry_msgs__msg__Twist mbot_vel_msg;
static std_msgs__msg__Float32MultiArray motor_vel_msg;
static std_msgs__msg__Float32MultiArray motor_pwm_msg;
static std_msgs__msg__Float32MultiArray analog_msg;

// Define ROS subscribers
static rcl_subscription_t cmd_vel_subscriber;
static rcl_subscription_t motor_vel_cmd_subscriber;
static rcl_subscription_t motor_pwm_cmd_subscriber;

// Define ROS subscription messages
static geometry_msgs__msg__Twist cmd_vel_msg;
static std_msgs__msg__Float32MultiArray motor_vel_cmd_msg;
static std_msgs__msg__Float32MultiArray motor_pwm_cmd_msg;

// Define ROS services
static rcl_service_t reset_odometry_service;
static rcl_service_t reset_encoders_service;

// Define ROS service messages
static std_srvs__srv__Trigger_Request reset_odometry_req;
static std_srvs__srv__Trigger_Response reset_odometry_res;
static std_srvs__srv__Trigger_Request reset_encoders_req;
static std_srvs__srv__Trigger_Response reset_encoders_res;

// Callback function prototypes
void cmd_vel_callback(const void * msg);
void motor_vel_cmd_callback(const void * msg);
void motor_pwm_cmd_callback(const void * msg);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void reset_odometry_callback(const void * request, void * response);
void reset_encoders_callback(const void * request, void * response);

// Function prototypes
static void mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, float* vx, float* vy, float* wz);
static void mbot_calculate_motor_vel(void);
static void mbot_read_encoders(void);
static void mbot_read_imu(void);
static void mbot_read_adc(void);
static void mbot_publish_state(void);

// Redirect the standard C library output functions to custom CDC implementation
int _write(int fd, const void *buf, size_t count) {
    // Use our custom function to write to CDC0
    dual_cdc_write_chars(0, buf, count);
    return count;
}

// Initialize microROS
int mbot_init_micro_ros(void) {
    // Set up transports
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );

    // Initialize allocator
    allocator = rcl_get_default_allocator();

    // Try to ping agent - non-blocking approach with short timeout
    rcl_ret_t ret = rmw_uros_ping_agent(100, 1);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR_AGENT_UNREACHABLE;
    }

    // From here, we proceed with initialization since agent is available
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    // Create node
    ret = rclc_node_init_default(&node, "pico_node", "", &support);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    // Initialize communication (publishers/subscribers)
    if (mbot_init_micro_ros_comm() != MBOT_OK) {
        return MBOT_ERROR;
    }
    
    // Create timer for periodic publishing
    ret = rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS((int)(MAIN_LOOP_PERIOD * 1000)),
        timer_callback,
        NULL);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    // Create executor
    ret = rclc_executor_init(&executor, &support.context, 6, &allocator);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    // Add timer to executor
    ret = rclc_executor_add_timer(&executor, &timer);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    // Add subscribers to executor
    ret = rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, 
                                        &cmd_vel_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    ret = rclc_executor_add_subscription(&executor, &motor_vel_cmd_subscriber, &motor_vel_cmd_msg, 
                                       &motor_vel_cmd_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    ret = rclc_executor_add_subscription(&executor, &motor_pwm_cmd_subscriber, &motor_pwm_cmd_msg, 
                                       &motor_pwm_cmd_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    // Add services to executor
    ret = rclc_executor_add_service(&executor, &reset_odometry_service, &reset_odometry_req, 
                                   &reset_odometry_res, &reset_odometry_callback);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    ret = rclc_executor_add_service(&executor, &reset_encoders_service, &reset_encoders_req, 
                                   &reset_encoders_res, &reset_encoders_callback);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    return MBOT_OK;
}

// Set up publishers and subscribers
int mbot_init_micro_ros_comm(void) {
    rcl_ret_t ret;
    
    // Initialize publishers
    ret = rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_publisher_init_default(
        &odom_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_publisher_init_default(
        &encoders_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoders"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_publisher_init_default(
        &mbot_vel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "mbot_vel"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_publisher_init_default(
        &motor_vel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_vel"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_publisher_init_default(
        &motor_pwm_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_pwm"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_publisher_init_default(
        &analog_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "analog"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    // Initialize message arrays for publishers
    // IMU message initialization
    imu_msg.header.frame_id.data = (char*)malloc(20 * sizeof(char));
    imu_msg.header.frame_id.capacity = 20;
    snprintf(imu_msg.header.frame_id.data, 20, "base_link");
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

    // Odometry message initialization
    odom_msg.header.frame_id.data = (char*)malloc(20 * sizeof(char));
    odom_msg.header.frame_id.capacity = 20;
    snprintf(odom_msg.header.frame_id.data, 20, "odom");
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.child_frame_id.data = (char*)malloc(20 * sizeof(char));
    odom_msg.child_frame_id.capacity = 20;
    snprintf(odom_msg.child_frame_id.data, 20, "base_footprint");
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);

    // Encoders message initialization
    encoders_msg.data.capacity = NUM_MOT_SLOTS;
    encoders_msg.data.size = NUM_MOT_SLOTS;
    encoders_msg.data.data = (int32_t*)malloc(NUM_MOT_SLOTS * sizeof(int32_t));

    // Motor velocity message initialization
    motor_vel_msg.data.capacity = NUM_MOT_SLOTS;
    motor_vel_msg.data.size = NUM_MOT_SLOTS;
    motor_vel_msg.data.data = (float*)malloc(NUM_MOT_SLOTS * sizeof(float));

    // Motor PWM message initialization
    motor_pwm_msg.data.capacity = NUM_MOT_SLOTS;
    motor_pwm_msg.data.size = NUM_MOT_SLOTS;
    motor_pwm_msg.data.data = (float*)malloc(NUM_MOT_SLOTS * sizeof(float));

    // Analog message initialization
    analog_msg.data.capacity = 4;  // 4 ADC channels
    analog_msg.data.size = 4;
    analog_msg.data.data = (float*)malloc(4 * sizeof(float));

    // Initialize subscribers 
    ret = rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    // Initialize motor velocity command subscriber
    ret = rclc_subscription_init_default(
        &motor_vel_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_vel_cmd"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    // Initialize motor PWM command subscriber
    ret = rclc_subscription_init_default(
        &motor_pwm_cmd_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_pwm_cmd"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    // Initialize the message arrays for motor commands
    motor_vel_cmd_msg.data.capacity = NUM_MOT_SLOTS;
    motor_vel_cmd_msg.data.size = NUM_MOT_SLOTS;
    motor_vel_cmd_msg.data.data = (float*)malloc(NUM_MOT_SLOTS * sizeof(float));

    motor_pwm_cmd_msg.data.capacity = NUM_MOT_SLOTS;
    motor_pwm_cmd_msg.data.size = NUM_MOT_SLOTS;
    motor_pwm_cmd_msg.data.data = (float*)malloc(NUM_MOT_SLOTS * sizeof(float));
    
    // Initialize services
    ret = rclc_service_init_default(
        &reset_odometry_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "reset_odometry"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_service_init_default(
        &reset_encoders_service,
        &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "reset_encoders"
    );
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    return MBOT_OK;
}


// Handle incoming ROS messages
int mbot_spin_micro_ros(void) {
    // Do not call tud_task here as it's called in the main loop
    static uint32_t spin_count = 0;
    
    // Log spin activity occasionally (every ~10 seconds)
    if (spin_count % 1000 == 0) {
        printf("microROS active - spin count: %lu\r\n", spin_count);
    }
    
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    if (ret != RCL_RET_OK) {
        // Only report serious errors, not TIMEOUT which is common
        if (ret != RCL_RET_TIMEOUT) {
            printf("microROS spin error: %d\r\n", ret);
            return MBOT_ERROR;
        }
    }
    
    spin_count++;
    return MBOT_OK;
}

// Callback for velocity commands
void cmd_vel_callback(const void * msg) {
    const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msg;
    mbot_cmd.vx = twist_msg->linear.x;
    mbot_cmd.vy = twist_msg->linear.y;
    mbot_cmd.wz = twist_msg->angular.z;
    mbot_cmd.drive_mode = MODE_MBOT_VEL;
    mbot_state.comms_active = true;
}

// Callback for motor velocity commands
void motor_vel_cmd_callback(const void * msg) {
    const std_msgs__msg__Float32MultiArray * vel_msg = (const std_msgs__msg__Float32MultiArray *)msg;
    for(int i = 0; i < NUM_MOT_SLOTS; i++) {
        mbot_cmd.wheel_vel[i] = vel_msg->data.data[i];
    }
    mbot_cmd.drive_mode = MODE_MOTOR_VEL_OL;
    mbot_state.comms_active = true;
}

// Callback for motor PWM commands
void motor_pwm_cmd_callback(const void * msg) {
    const std_msgs__msg__Float32MultiArray * pwm_msg = (const std_msgs__msg__Float32MultiArray *)msg;
    for(int i = 0; i < NUM_MOT_SLOTS; i++) {
        mbot_cmd.motor_pwm[i] = pwm_msg->data.data[i];
    }
    mbot_cmd.drive_mode = MODE_MOTOR_PWM;
    mbot_state.comms_active = true;
}

// Calculate motor velocities from encoder ticks
static void mbot_calculate_motor_vel(void) {
    float conversion = (1.0 / GEAR_RATIO) * (1.0 / ENCODER_RES) * 1E6f * 2.0 * PI;
    int64_t delta_time = mbot_state.timestamp_us - mbot_state.last_encoder_time;
    
    mbot_state.wheel_vel[MOT_L] = params.encoder_polarity[MOT_L] * 
        (conversion / delta_time) * mbot_state.encoder_delta_ticks[MOT_L];
    mbot_state.wheel_vel[MOT_R] = params.encoder_polarity[MOT_R] * 
        (conversion / delta_time) * mbot_state.encoder_delta_ticks[MOT_R];
        
    mbot_state.last_encoder_time = mbot_state.timestamp_us;
}

static void mbot_read_encoders(void) {
    // TODO: Implement actual encoder reading
    mbot_state.encoder_ticks[MOT_R] = 0;
    mbot_state.encoder_delta_ticks[MOT_R] = 0;
    mbot_state.encoder_ticks[MOT_L] = 0;
    mbot_state.encoder_delta_ticks[MOT_L] = 0;
}

static void mbot_read_imu(void) {
    // TODO: Implement actual IMU reading
    for(int i = 0; i < 3; i++) {
        mbot_state.imu_gyro[i] = 0;
        mbot_state.imu_accel[i] = 0;
        mbot_state.imu_mag[i] = 0;
        mbot_state.imu_rpy[i] = 0;
    }
    for(int i = 0; i < 4; i++) {
        mbot_state.imu_quat[i] = 0;
    }
}

static void mbot_read_adc(void) {
    // TODO: Implement actual ADC reading
    for(int i = 0; i < 4; i++) {
        mbot_state.analog_in[i] = 0;
    }
}

// Publish all robot state to ROS topics
static void mbot_publish_state(void) {
    rcl_ret_t ret;
    int64_t now = rmw_uros_epoch_nanos();

    // Publish IMU data
    imu_msg.header.stamp.sec = now / 1000000000;
    imu_msg.header.stamp.nanosec = now % 1000000000;
    
    imu_msg.angular_velocity.x = mbot_state.imu_gyro[0];
    imu_msg.angular_velocity.y = mbot_state.imu_gyro[1];
    imu_msg.angular_velocity.z = mbot_state.imu_gyro[2];
    imu_msg.linear_acceleration.x = mbot_state.imu_accel[0];
    imu_msg.linear_acceleration.y = mbot_state.imu_accel[1];
    imu_msg.linear_acceleration.z = mbot_state.imu_accel[2];
    imu_msg.orientation.w = mbot_state.imu_quat[0];
    imu_msg.orientation.x = mbot_state.imu_quat[1];
    imu_msg.orientation.y = mbot_state.imu_quat[2];
    imu_msg.orientation.z = mbot_state.imu_quat[3];
    
    ret = rcl_publish(&imu_publisher, &imu_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing IMU message: %d\r\n", ret);
    }

    // Publish odometry data
    odom_msg.header.stamp.sec = now / 1000000000;
    odom_msg.header.stamp.nanosec = now % 1000000000;
    
    odom_msg.pose.pose.position.x = mbot_state.odom_x;
    odom_msg.pose.pose.position.y = mbot_state.odom_y;
    odom_msg.pose.pose.position.z = 0.0;
    
    float cy = cos(mbot_state.odom_theta * 0.5);
    float sy = sin(mbot_state.odom_theta * 0.5);
    odom_msg.pose.pose.orientation.w = cy;
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = sy;
    
    odom_msg.twist.twist.linear.x = mbot_state.vx;
    odom_msg.twist.twist.linear.y = mbot_state.vy;
    odom_msg.twist.twist.angular.z = mbot_state.wz;
    
    ret = rcl_publish(&odom_publisher, &odom_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing odometry message: %d\r\n", ret);
    }

    // Publish encoder counts
    for (int i = 0; i < NUM_MOT_SLOTS; i++) {
        encoders_msg.data.data[i] = mbot_state.encoder_ticks[i];
    }
    ret = rcl_publish(&encoders_publisher, &encoders_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing encoder message: %d\r\n", ret);
    }

    // Publish motor velocities
    for (int i = 0; i < NUM_MOT_SLOTS; i++) {
        motor_vel_msg.data.data[i] = mbot_state.wheel_vel[i];
    }
    ret = rcl_publish(&motor_vel_publisher, &motor_vel_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing motor velocity message: %d\r\n", ret);
    }

    // Publish motor PWM values
    for (int i = 0; i < NUM_MOT_SLOTS; i++) {
        motor_pwm_msg.data.data[i] = mbot_state.motor_pwm[i];
    }
    ret = rcl_publish(&motor_pwm_publisher, &motor_pwm_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing motor PWM message: %d\r\n", ret);
    }

    // Publish analog inputs
    for (int i = 0; i < 4; i++) {
        analog_msg.data.data[i] = mbot_state.analog_in[i];
    }
    ret = rcl_publish(&analog_publisher, &analog_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("Error publishing analog input message: %d\r\n", ret);
    }
}

// Timer callback for periodic publishing
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    // Get current timestamp
    int64_t now = rmw_uros_epoch_nanos();
    mbot_state.timestamp_us = now / 1000;  // Convert to microseconds

    // Read all sensor data
    mbot_read_encoders();
    mbot_read_imu();
    mbot_read_adc();
    
    // Calculate velocities and update odometry
    mbot_calculate_motor_vel();
    mbot_calculate_diff_body_vel(
        mbot_state.wheel_vel[MOT_L],
        mbot_state.wheel_vel[MOT_R],
        &mbot_state.vx,
        &mbot_state.vy,
        &mbot_state.wz
    );
    mbot_calculate_odometry(
        mbot_state.vx,
        mbot_state.vy,
        mbot_state.wz,
        MAIN_LOOP_PERIOD,
        &mbot_state.odom_x,
        &mbot_state.odom_y,
        &mbot_state.odom_theta
    );

    // Publish all state data
    mbot_publish_state();
}

// Callback for reset odometry service
void reset_odometry_callback(const void * request, void * response) {
    // For future implementation
}

// Callback for reset encoders service
void reset_encoders_callback(const void * request, void * response) {
    // For future implementation
}

// Calculate body velocities from wheel velocities
static void mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, float* vx, float* vy, float* wz) {
    // Calculate forward velocity and angular velocity
    *vx = DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
    *vy = 0.0;
    *wz = DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);
}

/**
 * @brief Main entry point
 */
int main() {
    // Initialize stdio (sets up system clock and other defaults)
    stdio_init_all();
    
    // Initialize dual CDC interfaces
    dual_cdc_init();

    // Small delay to ensure USB is ready for early prints
    sleep_ms(3000);  

    printf("MBot Classic ROS - Starting up\r\n");
    
    // Initialize hardware components
    printf("Initializing motors...\r\n");
    mbot_motor_init(MOT_L);
    mbot_motor_init(MOT_R);

    printf("Initializing encoders...\r\n");
    mbot_encoder_init();

    printf("Initializing ADC...\r\n");
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);

    printf("Initializing FRAM...\r\n");
    mbot_init_fram();

    printf("Initializing IMU...\r\n");
    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_config.sample_rate = 200;
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);
    
    // Main loop
    printf("Entering main loop\r\n");
    uint32_t counter = 0;
    bool microros_enabled = false;
    
    while (1) {
        // Process USB tasks every loop iteration
        dual_cdc_task();
        
        // Handle microROS
        if (microros_enabled) {
            // If already initialized, spin
            if (mbot_spin_micro_ros() != MBOT_OK) {
                // If it fails, mark as disconnected
                microros_enabled = false;
            }
        } else {
            // Not connected - try to initialize
            int result = mbot_init_micro_ros();
            if (result == MBOT_OK) {
                microros_enabled = true;
                printf("microROS connected\r\n");
            }
        }
        
        // Status output once every 5 seconds
        if (counter % 500 == 0) {
            printf("MBot running - microROS: %s\r\n", 
                    microros_enabled ? "connected" : "waiting for agent");
        }
        
        counter++;
        sleep_ms(10);
    }
    
    return 0;
}