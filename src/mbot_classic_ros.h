/**
 * @file mbot_classic_ros.h
 * @brief MicroROS integration for MBot Classic
 */
#ifndef MBOT_CLASSIC_ROS_H
#define MBOT_CLASSIC_ROS_H

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <mbot/defs/mbot_params.h>
#include "config/mbot_classic_config.h"

// Message types
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_srvs/srv/trigger.h>

// Drive mode definitions
enum drive_modes
{
    MODE_MOTOR_PWM = 0,      // Direct PWM control
    MODE_MOTOR_VEL_OL = 1,   // Open-loop motor velocity control
    MODE_MBOT_VEL = 2        // Robot body velocity control
};

// Robot state structure
typedef struct {
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
    int64_t last_encoder_time;
} mbot_state_t;

// Command structure
typedef struct {
    float vx;
    float vy;
    float wz;
    float wheel_vel[NUM_MOT_SLOTS];
    float motor_pwm[NUM_MOT_SLOTS];
    int drive_mode;  // 0=PWM, 1=wheel vel, 2=body vel
} mbot_cmd_t;

/**
 * @brief Initialize microROS communication
 * 
 * @return int MBOT_OK on success, MBOT_ERROR otherwise
 */
int mbot_init_micro_ros(void);

/**
 * @brief Handle incoming ROS messages
 * 
 * @return int MBOT_OK on success, MBOT_ERROR otherwise
 */
int mbot_spin_micro_ros(void);

/**
 * @brief Set up ROS publishers and subscribers
 * 
 * @return int MBOT_OK on success, MBOT_ERROR otherwise
 */
int mbot_init_micro_ros_comm(void);

#endif /* MBOT_CLASSIC_ROS_H */ 