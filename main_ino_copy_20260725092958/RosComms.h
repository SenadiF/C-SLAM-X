#ifndef ROS_COMMS_H
#define ROS_COMMS_H

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <sensor_msgs/msg/laser_scan.h>

// Which physical robot this board is. MUST be unique per board - if two
// boards are both flashed with "robot1", they both end up on the same
// /robot1/cmd_vel, /robot1/imu, etc. topics, so both react to any command
// sent to either one (with a visible delay between them, since each board's
// executor/WiFi timing differs) instead of just the intended robot moving.
// Set this to "robot1" before flashing robot1's board, "robot2" before
// flashing robot2's board.
#define ROBOT_NAMESPACE "robot1"

// WiFi / Agent config
extern char* WIFI_SSID;
extern char* WIFI_PASSWORD;
extern char* AGENT_IP;
extern const uint16_t AGENT_PORT;

// ROS objects
extern rcl_allocator_t allocator;
extern rclc_support_t support;
extern rcl_node_t node;
extern rclc_executor_t executor;

// Messages / pubs / subs 
extern rcl_publisher_t imu_publisher;
extern sensor_msgs__msg__Imu imu_msg;

extern rcl_publisher_t encoder_publisher;
extern std_msgs__msg__Int32MultiArray encoder_msg;

extern rcl_subscription_t cmd_vel_subscriber;
extern geometry_msgs__msg__Twist cmd_vel_msg;

extern rcl_publisher_t buffered_lidar_publisher;
extern std_msgs__msg__Int32MultiArray buffered_lidar_msg;

extern rcl_publisher_t scan_publisher;
extern sensor_msgs__msg__LaserScan scan_msg;

void setupRosCore();

#endif