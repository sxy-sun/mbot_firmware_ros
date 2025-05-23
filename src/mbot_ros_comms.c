#include "mbot_ros_comms.h"
#include "mbot_classic_ros.h" // For mbot_state_t, mbot_cmd_t, and config defines
#include <string.h>            // For strlen, snprintf in message init

// Define ROS Objects (matching extern declarations in .h)
rcl_publisher_t imu_publisher;
rcl_publisher_t odom_publisher;
rcl_publisher_t encoders_publisher;
rcl_publisher_t mbot_vel_publisher;
rcl_publisher_t motor_vel_publisher;
rcl_publisher_t motor_pwm_publisher;
rcl_publisher_t analog_publisher;

sensor_msgs__msg__Imu imu_msg;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int32MultiArray encoders_msg;
geometry_msgs__msg__Twist mbot_vel_msg; // For publishing current mbot velocity
std_msgs__msg__Float32MultiArray motor_vel_msg;
std_msgs__msg__Float32MultiArray motor_pwm_msg;
std_msgs__msg__Float32MultiArray analog_msg;

rcl_subscription_t cmd_vel_subscriber;
rcl_subscription_t motor_vel_cmd_subscriber;
rcl_subscription_t motor_pwm_cmd_subscriber;

geometry_msgs__msg__Twist cmd_vel_msg_buffer; 
std_msgs__msg__Float32MultiArray motor_vel_cmd_msg_buffer;
std_msgs__msg__Float32MultiArray motor_pwm_cmd_msg_buffer;

rcl_service_t reset_odometry_service;
rcl_service_t reset_encoders_service;

std_srvs__srv__Trigger_Request reset_odometry_req;
std_srvs__srv__Trigger_Response reset_odometry_res;
std_srvs__srv__Trigger_Request reset_encoders_req;
std_srvs__srv__Trigger_Response reset_encoders_res;


int mbot_ros_comms_init_messages(rcl_allocator_t* allocator) {
    // IMU message initialization
    imu_msg.header.frame_id.data = (char*) malloc(20 * sizeof(char));
    if (!imu_msg.header.frame_id.data) return MBOT_ERROR; // Allocation check
    imu_msg.header.frame_id.capacity = 20;
    snprintf(imu_msg.header.frame_id.data, 20, "base_link");
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

    // Odometry message initialization
    odom_msg.header.frame_id.data = (char*) malloc(20 * sizeof(char));
    if (!odom_msg.header.frame_id.data) return MBOT_ERROR;
    odom_msg.header.frame_id.capacity = 20;
    snprintf(odom_msg.header.frame_id.data, 20, "odom");
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    
    odom_msg.child_frame_id.data = (char*) malloc(20 * sizeof(char));
    if (!odom_msg.child_frame_id.data) return MBOT_ERROR;
    odom_msg.child_frame_id.capacity = 20;
    snprintf(odom_msg.child_frame_id.data, 20, "base_footprint");
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);

    // Encoders message initialization (std_msgs__msg__Int32MultiArray)
    encoders_msg.data.capacity = NUM_MOT_SLOTS;
    encoders_msg.data.size = NUM_MOT_SLOTS; // Typically size is also set to capacity for fixed-size arrays
    encoders_msg.data.data = (int32_t*)malloc(NUM_MOT_SLOTS * sizeof(int32_t));
    if (!encoders_msg.data.data) return MBOT_ERROR;

    // Motor velocity message initialization (std_msgs__msg__Float32MultiArray)
    motor_vel_msg.data.capacity = NUM_MOT_SLOTS;
    motor_vel_msg.data.size = NUM_MOT_SLOTS;
    motor_vel_msg.data.data = (float*)malloc(NUM_MOT_SLOTS * sizeof(float));
    if (!motor_vel_msg.data.data) return MBOT_ERROR;

    // Motor PWM message initialization (std_msgs__msg__Float32MultiArray)
    motor_pwm_msg.data.capacity = NUM_MOT_SLOTS;
    motor_pwm_msg.data.size = NUM_MOT_SLOTS;
    motor_pwm_msg.data.data = (float*)malloc(NUM_MOT_SLOTS * sizeof(float));
    if (!motor_pwm_msg.data.data) return MBOT_ERROR;

    // Analog message initialization (std_msgs__msg__Float32MultiArray)
    analog_msg.data.capacity = 4;  // 4 ADC channels
    analog_msg.data.size = 4;
    analog_msg.data.data = (float*)malloc(4 * sizeof(float));
    if (!analog_msg.data.data) return MBOT_ERROR;
    
    // Initialize the message buffers for subscribers (Float32MultiArray)
    motor_vel_cmd_msg_buffer.data.capacity = NUM_MOT_SLOTS;
    motor_vel_cmd_msg_buffer.data.size = 0; // Size will be set by received message, init to 0
    motor_vel_cmd_msg_buffer.data.data = (float*)malloc(NUM_MOT_SLOTS * sizeof(float));
    if (!motor_vel_cmd_msg_buffer.data.data) return MBOT_ERROR;

    motor_pwm_cmd_msg_buffer.data.capacity = NUM_MOT_SLOTS;
    motor_pwm_cmd_msg_buffer.data.size = 0; // Size will be set by received message, init to 0
    motor_pwm_cmd_msg_buffer.data.data = (float*)malloc(NUM_MOT_SLOTS * sizeof(float));
    if (!motor_pwm_cmd_msg_buffer.data.data) return MBOT_ERROR;

    // Note: geometry_msgs__msg__Twist (cmd_vel_msg_buffer) doesn't need dynamic array allocation here for its members.
    // Its fields are fixed-size (geometry_msgs/Vector3 which are double[3]) or string (handled if needed).

    return MBOT_OK;
}

int mbot_ros_comms_init_publishers(rcl_node_t *node) {
    rcl_ret_t ret;
    printf("Initializing imu publisher...\n");
    ret = rclc_publisher_init_default(
        &imu_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu");
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    printf("Initializing odom publisher...\n");
    ret = rclc_publisher_init_default(
        &odom_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "odom");
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    printf("Initializing encoders publisher...\n");
    ret = rclc_publisher_init_default(
        &encoders_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoders");
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    printf("Initializing mbot_vel publisher...\n");
    ret = rclc_publisher_init_default(
        &mbot_vel_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "mbot_vel");
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    printf("Initializing motor_vel publisher...\n");
    ret = rclc_publisher_init_default(
        &motor_vel_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_vel");
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    printf("Initializing motor_pwm publisher...\n");
    ret = rclc_publisher_init_default(
        &motor_pwm_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_pwm");
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    printf("Initializing analog publisher...\n");
    ret = rclc_publisher_init_default(
        &analog_publisher,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "analog");
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    return MBOT_OK;
}

int mbot_ros_comms_init_subscribers(rcl_node_t *node) {
    rcl_ret_t ret;
    ret = rclc_subscription_init_default(
        &cmd_vel_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_subscription_init_default(
        &motor_vel_cmd_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_vel_cmd");
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_subscription_init_default(
        &motor_pwm_cmd_subscriber,
        node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "motor_pwm_cmd");
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    return MBOT_OK;
}

int mbot_ros_comms_init_services(rcl_node_t *node) {
    rcl_ret_t ret;
    ret = rclc_service_init_default(
        &reset_odometry_service,
        node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "reset_odometry");
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_service_init_default(
        &reset_encoders_service,
        node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "reset_encoders");
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    return MBOT_OK;
}

void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * twist_msg = (const geometry_msgs__msg__Twist *)msgin;
    mbot_cmd.vx = twist_msg->linear.x;
    mbot_cmd.vy = twist_msg->linear.y; // Assuming vy might be used, though typically 0 for diff drive
    mbot_cmd.wz = twist_msg->angular.z;
    mbot_cmd.drive_mode = MODE_MBOT_VEL;
    mbot_state.comms_active = true;
}

void motor_vel_cmd_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * vel_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    // Ensure received data does not exceed allocated buffer capacity
    size_t len = (vel_msg->data.size < NUM_MOT_SLOTS) ? vel_msg->data.size : NUM_MOT_SLOTS;
    for(size_t i = 0; i < len; i++) {
        mbot_cmd.wheel_vel[i] = vel_msg->data.data[i];
    }
    if (len < NUM_MOT_SLOTS) { // If partial data, zero out remaining slots if necessary
        for (size_t i = len; i < NUM_MOT_SLOTS; ++i) {
            mbot_cmd.wheel_vel[i] = 0.0f;
        }
    }
    mbot_cmd.drive_mode = MODE_MOTOR_VEL_OL;
    mbot_state.comms_active = true;
}

void motor_pwm_cmd_callback(const void * msgin) {
    const std_msgs__msg__Float32MultiArray * pwm_msg = (const std_msgs__msg__Float32MultiArray *)msgin;
    size_t len = (pwm_msg->data.size < NUM_MOT_SLOTS) ? pwm_msg->data.size : NUM_MOT_SLOTS;
    for(size_t i = 0; i < len; i++) {
        mbot_cmd.motor_pwm[i] = pwm_msg->data.data[i];
    }
    if (len < NUM_MOT_SLOTS) { // If partial data, zero out remaining slots
        for (size_t i = len; i < NUM_MOT_SLOTS; ++i) {
            mbot_cmd.motor_pwm[i] = 0.0f;
        }
    }
    mbot_cmd.drive_mode = MODE_MOTOR_PWM;
    mbot_state.comms_active = true;
}

void reset_odometry_callback(const void * request, void * response) {
    // TODO: Implement this
}

void reset_encoders_callback(const void * request, void * response) {
    // TODO: Implement this
}

int mbot_ros_comms_add_to_executor(rclc_executor_t *executor) {
    rcl_ret_t ret;

    // Add subscribers
    ret = rclc_executor_add_subscription(executor, &cmd_vel_subscriber, &cmd_vel_msg_buffer, 
                                        &cmd_vel_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    ret = rclc_executor_add_subscription(executor, &motor_vel_cmd_subscriber, &motor_vel_cmd_msg_buffer, 
                                       &motor_vel_cmd_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    ret = rclc_executor_add_subscription(executor, &motor_pwm_cmd_subscriber, &motor_pwm_cmd_msg_buffer, 
                                       &motor_pwm_cmd_callback, ON_NEW_DATA);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    // Add services
    ret = rclc_executor_add_service(executor, &reset_odometry_service, &reset_odometry_req, 
                                   &reset_odometry_res, &reset_odometry_callback);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    
    ret = rclc_executor_add_service(executor, &reset_encoders_service, &reset_encoders_req, 
                                   &reset_encoders_res, &reset_encoders_callback);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    return MBOT_OK;
} 