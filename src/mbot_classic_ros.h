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

/**
 * @brief Initialize microROS communication
 * 
 * @return int MBOT_OK on success, MBOT_ERROR otherwise
 */
int mbot_init_micro_ros(void);

/**
 * @brief Publish MBot data to ROS topics
 * 
 * @return int MBOT_OK on success, MBOT_ERROR otherwise
 */
int mbot_publish_micro_ros(void);

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