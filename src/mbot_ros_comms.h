#ifndef MBOT_ROS_COMMS_H
#define MBOT_ROS_COMMS_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_srvs/srv/trigger.h>

// Forward declare mbot_state_t and mbot_cmd_t if their full definitions aren't needed here
// or include the header that defines them (e.g. "mbot_classic_ros.h" might be too circular)
// For now, assuming callbacks in mbot_ros_comms.c will include "mbot_classic_ros.h"
// to get access to mbot_state and mbot_cmd.

// Extern declarations for ROS objects

// Publishers
extern rcl_publisher_t imu_publisher;
extern rcl_publisher_t odom_publisher;
extern rcl_publisher_t encoders_publisher;
extern rcl_publisher_t mbot_vel_publisher;
extern rcl_publisher_t motor_vel_publisher;
extern rcl_publisher_t motor_pwm_publisher;
extern rcl_publisher_t analog_publisher;

// Published Messages (these are filled and published)
extern sensor_msgs__msg__Imu imu_msg;
extern nav_msgs__msg__Odometry odom_msg;
extern std_msgs__msg__Int32MultiArray encoders_msg;
extern geometry_msgs__msg__Twist mbot_vel_msg; // For publishing current mbot velocity
extern std_msgs__msg__Float32MultiArray motor_vel_msg;
extern std_msgs__msg__Float32MultiArray motor_pwm_msg;
extern std_msgs__msg__Float32MultiArray analog_msg;

// Subscribers
extern rcl_subscription_t cmd_vel_subscriber;
extern rcl_subscription_t motor_vel_cmd_subscriber;
extern rcl_subscription_t motor_pwm_cmd_subscriber;

// Subscription Message Buffers (these are filled by incoming messages)
extern geometry_msgs__msg__Twist cmd_vel_msg_buffer; // Renamed to avoid clash with published mbot_vel_msg
extern std_msgs__msg__Float32MultiArray motor_vel_cmd_msg_buffer; // Renamed
extern std_msgs__msg__Float32MultiArray motor_pwm_cmd_msg_buffer; // Renamed

// Services
extern rcl_service_t reset_odometry_service;
extern rcl_service_t reset_encoders_service;

// Service Message Buffers (Request & Response)
extern std_srvs__srv__Trigger_Request reset_odometry_req;
extern std_srvs__srv__Trigger_Response reset_odometry_res;
extern std_srvs__srv__Trigger_Request reset_encoders_req;
extern std_srvs__srv__Trigger_Response reset_encoders_res;


// Initialization functions
int mbot_ros_comms_init_messages(rcl_allocator_t* allocator); // Allocator might be needed for messages if not preallocated
int mbot_ros_comms_init_publishers(rcl_node_t *node);
int mbot_ros_comms_init_subscribers(rcl_node_t *node);
int mbot_ros_comms_init_services(rcl_node_t *node);

// Callback function prototypes
void cmd_vel_callback(const void * msgin);
void motor_vel_cmd_callback(const void * msgin);
void motor_pwm_cmd_callback(const void * msgin);
void reset_odometry_callback(const void * request, void * response);
void reset_encoders_callback(const void * request, void * response);

// Helper to add comms to executor
int mbot_ros_comms_add_to_executor(rclc_executor_t *executor);

#endif // MBOT_ROS_COMMS_H 