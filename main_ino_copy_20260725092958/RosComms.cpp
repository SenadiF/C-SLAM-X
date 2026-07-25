#include "RosComms.h"
#include <rosidl_runtime_c/string_functions.h>

const char* WIFI_SSID = "Sena";
const char* WIFI_PASSWORD = "Devanga@123";
IPAddress AGENT_IP(172, 20, 10, 6);
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

  encoder_msg.data.data = (int32_t *)malloc(2 * sizeof(int32_t));
  encoder_msg.data.size = 2;
  encoder_msg.data.capacity = 2;
}