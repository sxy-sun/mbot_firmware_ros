/**
 * @file mbot_classic_ros.c
 * @brief MicroROS integration for MBot Classic
 */
#include "mbot_classic_ros.h"

#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

// Message types
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_srvs/srv/trigger.h>
#include <rmw_microros/time_sync.h>

// Hardware includes
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/clocks.h"
#include "pico_uart_transports.h"
#include "config/mbot_classic_config.h"
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>

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

// State variables maintained for the robot
static struct {
    int64_t timestamp_us;
    // Odometry
    float odom_x;
    float odom_y;
    float odom_theta;
    // Body velocity
    float vx;
    float vy;
    float wz;
    // Wheel state
    float wheel_vel[NUM_MOT_SLOTS];
    float wheel_pos[NUM_MOT_SLOTS];
    int32_t encoder_ticks[NUM_MOT_SLOTS];
    int32_t encoder_delta_ticks[NUM_MOT_SLOTS];
    // Motor control
    float motor_pwm[NUM_MOT_SLOTS];
    // IMU data
    float imu_gyro[3];  // x, y, z
    float imu_accel[3]; // x, y, z
    float imu_quat[4];  // w, x, y, z
    float imu_mag[3];   // Magnetometer data
    float imu_rpy[3];   // Roll, pitch, yaw
    // Analog inputs
    float analog_in[4]; // 4 ADC channels
    // Status
    bool comms_active;
} mbot_state = {0};

// Desired state (from commands)
static struct {
    float vx;
    float vy;
    float wz;
    float wheel_vel[NUM_MOT_SLOTS];
    float motor_pwm[NUM_MOT_SLOTS];
    int drive_mode;  // 0=PWM, 1=wheel vel, 2=body vel
} mbot_cmd = {0};

// Callback function prototypes
void cmd_vel_callback(const void * msg);
void motor_vel_callback(const void * msg);
void motor_pwm_callback(const void * msg);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void reset_odometry_callback(const void * request, void * response);
void reset_encoders_callback(const void * request, void * response);

extern mbot_params_t params;

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

    // Wait for agent successful ping
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK) {
        printf("Failed to ping agent, exiting\r\n");
        return MBOT_ERROR;
    }

    printf("Connected to micro-ROS agent!\r\n");
    
    // Initialize support
    ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize support\r\n");
        return MBOT_ERROR;
    }
    
    // Create node
    ret = rclc_node_init_default(&node, "mbot_node", "", &support);
    if (ret != RCL_RET_OK) {
        printf("Failed to create node\r\n");
        return MBOT_ERROR;
    }
    
    // Initialize communication (publishers/subscribers)
    if (mbot_init_micro_ros_comm() != MBOT_OK) {
        printf("Failed to initialize ROS communication\r\n");
        return MBOT_ERROR;
    }
    
    // Create timer for periodic publishing (100ms = 10Hz)
    ret = rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);
    if (ret != RCL_RET_OK) {
        printf("Failed to create timer\r\n");
        return MBOT_ERROR;
    }
    
    // Create executor with enough handles for our subscribers and timer
    ret = rclc_executor_init(&executor, &support.context, 6, &allocator);
    if (ret != RCL_RET_OK) {
        printf("Failed to initialize executor\r\n");
        return MBOT_ERROR;
    }
    
    // Add timer to executor
    ret = rclc_executor_add_timer(&executor, &timer);
    if (ret != RCL_RET_OK) {
        printf("Failed to add timer to executor\r\n");
        return MBOT_ERROR;
    }
    
    // Add subscribers to executor
    ret = rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, 
                                        &cmd_vel_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    ret = rclc_executor_add_subscription(&executor, &motor_vel_cmd_subscriber, &motor_vel_cmd_msg, 
                                        &motor_vel_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    ret = rclc_executor_add_subscription(&executor, &motor_pwm_cmd_subscriber, &motor_pwm_cmd_msg, 
                                        &motor_pwm_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    // Add services to executor
    ret = rclc_executor_add_service(&executor, &reset_odometry_service, &reset_odometry_req, 
                                  &reset_odometry_res, &reset_odometry_callback);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    ret = rclc_executor_add_service(&executor, &reset_encoders_service, &reset_encoders_req, 
                                  &reset_encoders_res, &reset_encoders_callback);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    return MBOT_OK;
}

// Set up publishers and subscribers
int mbot_init_micro_ros_comm(void) {
    rcl_ret_t ret;
    
    // Initialize publishers
    // For future implementation
    
    // Initialize subscribers 
    // For future implementation
    
    // Initialize services
    // For future implementation
    
    return MBOT_OK;
}

// Publish MBot data to ROS topics
int mbot_publish_micro_ros(void) {
    // For future implementation
    return MBOT_OK;
}

// Handle incoming ROS messages
int mbot_spin_micro_ros(void) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    return MBOT_OK;
}

// Callback for velocity commands
void cmd_vel_callback(const void * msg) {
    // For future implementation
}

// Callback for motor velocity commands
void motor_vel_callback(const void * msg) {
    // For future implementation
}

// Callback for motor PWM commands
void motor_pwm_callback(const void * msg) {
    // For future implementation
}

// Timer callback for periodic publishing
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    // For future implementation
}

// Callback for reset odometry service
void reset_odometry_callback(const void * request, void * response) {
    // For future implementation
}

// Callback for reset encoders service
void reset_encoders_callback(const void * request, void * response) {
    // For future implementation
}

/**
 * @brief Main entry point
 */
int main() {
    printf("\r********************************\r\n");
    printf("\r* MBot Classic ROS Firmware    *\r\n");
    printf("\r********************************\r\n");

    // Initialize the Pico hardware
    bi_decl(bi_program_description("MBot Classic firmware with microROS"));
    if(!set_sys_clock_khz(125000, true)) {
        printf("ERROR: Cannot set system clock\r\n");
        return -1;
    }
    stdio_init_all();
    sleep_ms(500);
    printf("\r\nMBot Booting Up!\r\n");
    
    // Initialize hardware components
    sleep_ms(1000);
    
    // Initialize motors
    printf("Initializing motors...\r\n");
    mbot_motor_init(MOT_L);
    mbot_motor_init(MOT_R);
    
    // Initialize encoders
    printf("Initializing encoders...\r\n");
    mbot_encoder_init();
    
    // Initialize IMU and other hardware
    // More initialization will be implemented
    
    // Initialize microROS
    printf("Initializing microROS...\r\n");
    if (mbot_init_micro_ros() != MBOT_OK) {
        printf("Failed to initialize microROS\r\n");
        return -1;
    }
    
    printf("Starting main loop...\r\n");
    
    while (1) {
        // Process incoming messages and services
        mbot_spin_micro_ros();
        
        // Small delay to prevent CPU hogging
        sleep_ms(10);
    }
    
    return 0;
} 