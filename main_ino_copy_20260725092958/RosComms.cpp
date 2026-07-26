#include "RosComms.h"
#include <rosidl_runtime_c/string_functions.h>

char* WIFI_SSID = (char*)"Sena";
char* WIFI_PASSWORD = (char*)"Devanga@123";
char* AGENT_IP = (char*)"172.20.10.6";
const uint16_t AGENT_PORT = 8888;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t encoder_publisher;
std_msgs__msg__Int32MultiArray encoder_msg;

rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_publisher_t buffered_lidar_publisher;
std_msgs__msg__Int32MultiArray buffered_lidar_msg;

rcl_publisher_t scan_publisher;
sensor_msgs__msg__LaserScan scan_msg;

void setupRosCore() {
  set_microros_wifi_transports(
      (char*)WIFI_SSID, (char*)WIFI_PASSWORD,
      AGENT_IP, AGENT_PORT
  );
  delay(2000);

  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "imu_node", "robot1", &support);

  rclc_publisher_init_default(
      &imu_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu"
  );

  buffered_lidar_msg.data.data = (int32_t *)malloc(36 * sizeof(int32_t));
  buffered_lidar_msg.data.size = 36;
  buffered_lidar_msg.data.capacity = 36;

  rclc_publisher_init_default(
    &buffered_lidar_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "buffered_scan"
);

  encoder_msg.data.data = (int32_t *)malloc(2 * sizeof(int32_t));
  encoder_msg.data.size = 2;
  encoder_msg.data.capacity = 2;

  rosidl_runtime_c__String__assign(&scan_msg.header.frame_id, "lidar_link");

  scan_msg.ranges.data = (float *)malloc(360 * sizeof(float));
  scan_msg.ranges.size = 360;
  scan_msg.ranges.capacity = 360;

  scan_msg.intensities.data = NULL; 
  scan_msg.intensities.size = 0;
  scan_msg.intensities.capacity = 0;

  scan_msg.angle_min = 0.0;
  scan_msg.angle_max = 2 * PI;
  scan_msg.angle_increment = (2 * PI) / 360.0;
  scan_msg.time_increment = 
  scan_msg.range_min = 0.05;  
  scan_msg.range_max = 12.0;  

rclc_publisher_init_default(
    &scan_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan"
);
}