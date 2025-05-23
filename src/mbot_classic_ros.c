/**
 * @file mbot_classic_ros.c
 * @brief MicroROS integration for MBot Classic
 */
#include <math.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/multicore.h>
#include "pico/time.h"
#include <hardware/clocks.h>
#include <hardware/adc.h>

// mbotlib
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <mbot/fram/fram.h>
#include <mbot/imu/imu.h>
#include <mbot/utils/utils.h>

// mbot_classic_ros
#include "mbot_classic_ros.h"
#include "mbot_odometry.h"
#include "mbot_ros_comms.h"
#include "mbot_print.h"

// comms
#include <comms/pico_uart_transports.h>
#include <comms/dual_cdc.h>

// robot control
#include <rc/math/filter.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// Global state variables
mbot_state_t mbot_state = {0};
mbot_cmd_t mbot_cmd = {0};
mbot_params_t params;
mbot_bhy_config_t mbot_imu_config;
mbot_bhy_data_t mbot_imu_data;
static bool enable_pwm_lpf = true;
rc_filter_t mbot_left_pwm_lpf;
rc_filter_t mbot_right_pwm_lpf;

// Global MicroROS objects
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

// Timer for periodic publishing
static rcl_timer_t ros_publish_timer;
static repeating_timer_t mbot_loop_timer;

static agent_state_t agent_state = WAITING_AGENT;

int mbot_init_micro_ros(void);
int mbot_spin_micro_ros(void);
static void mbot_publish_state(void);
static bool mbot_loop(repeating_timer_t *rt);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// helper functions
static int mbot_init_hardware(void);
static void mbot_read_imu(void);
static void mbot_read_encoders(void);
static void mbot_read_adc(void);
static void mbot_calculate_motor_vel(void);
static void mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, float* vx, float* vy, float* wz);
static void print_mbot_params(const mbot_params_t* params);
int mbot_destroy_micro_ros(void);

// Initialize microROS
int mbot_init_micro_ros(void) {
    printf("Getting default allocator...\n");
    allocator = rcl_get_default_allocator();

    printf("Initializing rclc support...\n");
    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        printf("rclc_support_init failed: %d\n", ret);
        return MBOT_ERROR;
    }
    
    printf("Creating node...\n");
    ret = rclc_node_init_default(&node, "pico_node", "", &support);
    if (ret != RCL_RET_OK) {
        printf("rclc_node_init_default failed: %d\n", ret);
        return MBOT_ERROR;
    }
    
    // Initialize ROS messages, publishers, subscribers, services using the new module
    printf("Initializing ROS messages...\n");
    ret = mbot_ros_comms_init_messages(&allocator);
    if (ret != MBOT_OK) return MBOT_ERROR;

    printf("Initializing publishers...\n");
    ret = mbot_ros_comms_init_publishers(&node);
    if (ret != MBOT_OK) return MBOT_ERROR;

    printf("Initializing subscribers...\n");
    ret = mbot_ros_comms_init_subscribers(&node);
    if (ret != MBOT_OK) return MBOT_ERROR;
    
    printf("Initializing services...\n");
    ret = mbot_ros_comms_init_services(&node);
    if (ret != MBOT_OK) return MBOT_ERROR;
    
    printf("Initializing timer...\n");
    ret = rclc_timer_init_default2(
        &ros_publish_timer,
        &support,
        RCL_MS_TO_NS((int)(ROS_TIMER_PERIOD * 1000)),
        timer_callback,
        NULL);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    printf("Initializing executor...\n");
    ret = rclc_executor_init(&executor, &support.context, 6, &allocator);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    printf("Adding timer to executor...\n");
    ret = rclc_executor_add_timer(&executor, &ros_publish_timer);
    if (ret != RCL_RET_OK) {
        return MBOT_ERROR;
    }
    
    printf("Adding subscribers and services to executor...\n");
    ret = mbot_ros_comms_add_to_executor(&executor);
    if (ret != MBOT_OK) return MBOT_ERROR;
    
    return MBOT_OK;
}

// Handle incoming ROS messages
int mbot_spin_micro_ros(void) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
        printf("microROS spin error: %d\r\n", ret);
        return MBOT_ERROR;
    }
    
    return MBOT_OK;
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

// Main robot logic loop, runs at MAIN_LOOP_HZ (called by hardware timer)
static bool mbot_loop(repeating_timer_t *rt) {
    mbot_read_encoders();    
    mbot_read_imu();
    mbot_read_adc();

    mbot_state.timestamp_us = time_us_64();

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
    
    // TODO: Drive the motors here, only run when we have 2 way com

    // TODO: check for time out here, if so, stop the motors
    // TODO: Consider how mbot_state.comms_active is managed for timeout.
    //       It's set true in ROS message callbacks. It needs a mechanism to become false.
    return true; // Keep timer running
}

// Timer callback for periodic ROS publishing (runs at ROS_TIMER_HZ)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    mbot_publish_state();
}

/**
 * @brief Main entry point
 */
int main() {
    // Initialize Dual CDC and stdio
    stdio_init_all();
    dual_cdc_init();
    mbot_wait_ms(2000);
    
    printf("\r\nMBot Classic Firmware (ROS2)\r\n");
    printf("--------------------------------\r\n");

    mbot_init_hardware();

    mbot_read_fram(0, sizeof(params), (uint8_t*)&params);

    printf("\nCalibration Parameters:\n");
    print_mbot_params(&params);
    mbot_wait_ms(1000);

    int validate_status = validate_mbot_classic_FRAM_data(&params, MOT_L, MOT_R, MOT_UNUSED);
    if (validate_status < 0){
        printf("Failed to validate FRAM Data! Error code: %d\n", validate_status);
    }

    printf("\nStarting MBot Loop...\n");
    mbot_state.last_encoder_time = time_us_64(); 
    if (!add_repeating_timer_ms((int32_t)(MAIN_LOOP_PERIOD * 1000.0f), mbot_loop, NULL, &mbot_loop_timer)){
        printf("Failed to add control loop timer! Halting.\r\n");
        while(1) {tight_loop_contents();}
    }

    rmw_ret_t rmw_ret = rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    if (rmw_ret != RMW_RET_OK) {
        printf("rmw_uros_set_custom_transport failed: %d\n", rmw_ret);
        return MBOT_ERROR;
    }

    printf("Done Booting Up!\n");
    fflush(stdout); 

    mbot_state.comms_active = false;
    static int64_t last_200ms_time = 0;
    static agent_state_t last_agent_state = -1;
    while (1) {
        dual_cdc_task(); // Keep USB alive - CRITICAL

        int64_t now = time_us_64();
        if (now - last_200ms_time > 200000) { // 200ms interval
            last_200ms_time = now;

            switch (agent_state) {
                case WAITING_AGENT:
                    if (last_agent_state != agent_state) {
                        printf("\n[STATE] WAITING_AGENT\n");
                        last_agent_state = agent_state;
                    }
                    rcl_ret_t ping_ret = rmw_uros_ping_agent(100, 1);
                    if (ping_ret == RMW_RET_OK) {
                        agent_state = AGENT_AVAILABLE;
                    }
                    break;

                case AGENT_AVAILABLE:
                    if (last_agent_state != agent_state) {
                        printf("\n[STATE] AGENT_AVAILABLE\n");
                        last_agent_state = agent_state;
                    }
                    if (mbot_init_micro_ros() == MBOT_OK) {
                        agent_state = AGENT_CONNECTED;
                    } else {
                        printf("[ERROR] micro-ROS initialization failed!\n");
                        mbot_destroy_micro_ros();
                        agent_state = WAITING_AGENT;
                    }
                    break;

                case AGENT_CONNECTED:
                    if (last_agent_state != agent_state) {
                        printf("\n[STATE] AGENT_CONNECTED\n");
                        last_agent_state = agent_state;
                    }
                    if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                        agent_state = AGENT_DISCONNECTED;
                    } else {
                        if (mbot_spin_micro_ros() != MBOT_OK) {
                            agent_state = AGENT_DISCONNECTED;
                        }
                        mbot_state.comms_active = true;
                        mbot_print_state(&mbot_state);
                    }
                    break;

                case AGENT_DISCONNECTED:
                    if (last_agent_state != agent_state) {
                        printf("\n[STATE] AGENT_DISCONNECTED\n");
                        last_agent_state = agent_state;
                    }
                    mbot_destroy_micro_ros();
                    mbot_state.comms_active = false;
                    agent_state = WAITING_AGENT;
                    break;
            }
            fflush(stdout); 
        }
    }
    return 0;
}

/******************************************************
 * Helper Functions
 * ----------------------------------------------------
 * These functions are used internally by the main control functions.
 * They are not intended for modification by students. These functions
 * provide lower-level control and utility support.
 ******************************************************/
static int mbot_init_hardware(void){
    printf("Initializing Hardwares...\n");
    // Initialize Motors
    mbot_motor_init(MOT_L);
    mbot_motor_init(MOT_R);
    mbot_encoder_init();

    // Initialize the IMU 
    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_config.sample_rate = 200;
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);

    // Initialize ADC
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);

    // Initialize PWM LPFs for smoother motion
    mbot_left_pwm_lpf = rc_filter_empty();
    mbot_right_pwm_lpf = rc_filter_empty();
    rc_filter_first_order_lowpass(&mbot_left_pwm_lpf, MAIN_LOOP_PERIOD, 4.0 * MAIN_LOOP_PERIOD);
    rc_filter_first_order_lowpass(&mbot_right_pwm_lpf, MAIN_LOOP_PERIOD, 4.0 * MAIN_LOOP_PERIOD);

    // Initialize FRAM
    mbot_init_fram();
    return MBOT_OK;
}

static void mbot_read_imu(void) {
    for(int i = 0; i < 3; i++) {
        mbot_state.imu_gyro[i] = mbot_imu_data.gyro[i];
        mbot_state.imu_accel[i] = mbot_imu_data.accel[i];
        mbot_state.imu_mag[i] = mbot_imu_data.mag[i];
        mbot_state.imu_rpy[i] = mbot_imu_data.rpy[i];
    }
    for(int i = 0; i < 4; i++) {
        mbot_state.imu_quat[i] = mbot_imu_data.quat[i];
    }
}

static void mbot_read_encoders(void) {
    int64_t now = time_us_64();

    // Calculate actual delta time since last encoder read
    // mbot_state.last_encoder_time is initialized in main() before the loop starts
    mbot_state.encoder_delta_t = now - mbot_state.last_encoder_time;
    
    // If dt is zero or negative (e.g. time_us_64 wraps or error), use nominal period
    if (mbot_state.encoder_delta_t <= 0) {
        mbot_state.encoder_delta_t = ((int64_t)(MAIN_LOOP_PERIOD * 1000000.0f));
    }

    mbot_state.last_encoder_time = now; // Update for the next cycle
    mbot_state.encoder_ticks[MOT_L] = mbot_encoder_read_count(MOT_L);
    mbot_state.encoder_ticks[MOT_R] = mbot_encoder_read_count(MOT_R);
    mbot_state.encoder_delta_ticks[MOT_L] = mbot_encoder_read_delta(MOT_L);
    mbot_state.encoder_delta_ticks[MOT_R] = mbot_encoder_read_delta(MOT_R);
}

static void mbot_read_adc(void) {
    const float conversion_factor = 3.0f / (1 << 12);
    int16_t raw;
    for(int i = 0; i < 4; i++) {
        adc_select_input(i);
        raw = adc_read();
        mbot_state.analog_in[i] = conversion_factor * raw;
    }
    // last channel is battery voltage (has 5x divider)
    mbot_state.analog_in[3] = 5.0f * conversion_factor * raw;
}

static void mbot_calculate_motor_vel(void) {
    float conversion = (1.0f / GEAR_RATIO) * (1.0f / ENCODER_RES) * 1E6f * 2.0f * PI;
    int64_t delta_t = mbot_state.encoder_delta_t; // Use the corrected delta_time
    
    if (delta_t <= 0) { /* Avoid division by zero or invalid dt */ 
        mbot_state.wheel_vel[MOT_L] = 0.0f;
        mbot_state.wheel_vel[MOT_R] = 0.0f;
        return; 
    }

    mbot_state.wheel_vel[MOT_L] = params.encoder_polarity[MOT_L] * 
        (conversion / delta_t) * mbot_state.encoder_delta_ticks[MOT_L];
    mbot_state.wheel_vel[MOT_R] = params.encoder_polarity[MOT_R] * 
        (conversion / delta_t) * mbot_state.encoder_delta_ticks[MOT_R];
}

static void mbot_calculate_diff_body_vel(float wheel_left_vel, float wheel_right_vel, float* vx, float* vy, float* wz) {
    // Calculate forward velocity and angular velocity
    *vx = DIFF_WHEEL_RADIUS * (wheel_left_vel - wheel_right_vel) / 2.0f;
    *vy = 0.0;
    *wz = DIFF_WHEEL_RADIUS * (-wheel_left_vel - wheel_right_vel) / (2.0f * DIFF_BASE_RADIUS);
}

static void print_mbot_params(const mbot_params_t* params) {
    printf("Motor Polarity: %d %d\n", params->motor_polarity[MOT_L], params->motor_polarity[MOT_R]);
    printf("Encoder Polarity: %d %d\n", params->encoder_polarity[MOT_L], params->encoder_polarity[MOT_R]);
    printf("Positive Slope: %f %f\n", params->slope_pos[MOT_L], params->slope_pos[MOT_R]);
    printf("Positive Intercept: %f %f\n", params->itrcpt_pos[MOT_L], params->itrcpt_pos[MOT_R]);
    printf("Negative Slope: %f %f\n", params->slope_neg[MOT_L], params->slope_neg[MOT_R]);
    printf("Negative Intercept: %f %f\n", params->itrcpt_neg[MOT_L], params->itrcpt_neg[MOT_R]);
}

int mbot_destroy_micro_ros(void) {
    printf("\nDestroying micro-ROS entities...\n");
    rcl_ret_t ret;
    int error = 0;

    // Destroy executor
    ret = rclc_executor_fini(&executor);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Failed to destroy executor: %d\n", ret);
        error = 1;
    }

    // Destroy timer
    ret = rcl_timer_fini(&ros_publish_timer);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Failed to destroy timer: %d\n", ret);
        error = 1;
    }

    // Destroy publishers
    ret = rcl_publisher_fini(&imu_publisher, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy imu_publisher: %d\n", ret);
    ret = rcl_publisher_fini(&odom_publisher, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy odom_publisher: %d\n", ret);
    ret = rcl_publisher_fini(&encoders_publisher, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy encoders_publisher: %d\n", ret);
    ret = rcl_publisher_fini(&motor_vel_publisher, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy motor_vel_publisher: %d\n", ret);
    ret = rcl_publisher_fini(&motor_pwm_publisher, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy motor_pwm_publisher: %d\n", ret);
    ret = rcl_publisher_fini(&analog_publisher, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy analog_publisher: %d\n", ret);
    ret = rcl_publisher_fini(&mbot_vel_publisher, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy mbot_vel_publisher: %d\n", ret);

    // Destroy subscribers
    ret = rcl_subscription_fini(&cmd_vel_subscriber, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy cmd_vel_subscriber: %d\n", ret);
    ret = rcl_subscription_fini(&motor_vel_cmd_subscriber, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy motor_vel_cmd_subscriber: %d\n", ret);
    ret = rcl_subscription_fini(&motor_pwm_cmd_subscriber, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy motor_pwm_cmd_subscriber: %d\n", ret);

    // Destroy services
    ret = rcl_service_fini(&reset_odometry_service, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy reset_odometry_service: %d\n", ret);
    ret = rcl_service_fini(&reset_encoders_service, &node);
    if (ret != RCL_RET_OK) printf("[ERROR] Failed to destroy reset_encoders_service: %d\n", ret);

    // Destroy node
    ret = rcl_node_fini(&node);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Failed to destroy node: %d\n", ret);
        error = 1;
    }

    // Destroy support
    ret = rclc_support_fini(&support);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Failed to destroy support: %d\n", ret);
        error = 1;
    }

    // Free dynamically allocated message memory (frame_id, child_frame_id, data arrays)
    if (imu_msg.header.frame_id.data) { free(imu_msg.header.frame_id.data); imu_msg.header.frame_id.data = NULL; }
    if (odom_msg.header.frame_id.data) { free(odom_msg.header.frame_id.data); odom_msg.header.frame_id.data = NULL; }
    if (odom_msg.child_frame_id.data) { free(odom_msg.child_frame_id.data); odom_msg.child_frame_id.data = NULL; }
    if (encoders_msg.data.data) { free(encoders_msg.data.data); encoders_msg.data.data = NULL; }
    if (motor_vel_msg.data.data) { free(motor_vel_msg.data.data); motor_vel_msg.data.data = NULL; }
    if (motor_pwm_msg.data.data) { free(motor_pwm_msg.data.data); motor_pwm_msg.data.data = NULL; }
    if (analog_msg.data.data) { free(analog_msg.data.data); analog_msg.data.data = NULL; }
    if (motor_vel_cmd_msg_buffer.data.data) { free(motor_vel_cmd_msg_buffer.data.data); motor_vel_cmd_msg_buffer.data.data = NULL; }
    if (motor_pwm_cmd_msg_buffer.data.data) { free(motor_pwm_cmd_msg_buffer.data.data); motor_pwm_cmd_msg_buffer.data.data = NULL; }

    if (error) return MBOT_ERROR;
    return MBOT_OK;
}