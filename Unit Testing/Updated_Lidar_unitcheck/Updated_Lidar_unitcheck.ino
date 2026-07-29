#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <rosidl_runtime_c/string_functions.h>

#include "LidarParserSTL.h"   

#define LIDAR_RX_PIN 16
#define LIDAR_TX_PIN 17
#define SerialLidar Serial2

char* WIFI_SSID = (char*)"Sena";
char* WIFI_PASSWORD = (char*)"Devanga@123";
char* AGENT_IP = (char*)"172.20.10.6";
const uint16_t AGENT_PORT = 8888;

uint16_t lidarDistances[360] = {0};
LidarParserSTL lidarParser;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t scan_publisher;
sensor_msgs__msg__LaserScan scan_msg;

const uint32_t PUBLISH_PERIOD_MS = 100;
unsigned long last_publish_time = 0;

void onLidarPoint(const LidarResultData& point, void* ref) {
  int angleDeg = ((int)point.angle) % 360;
  if (angleDeg < 0) angleDeg += 360;

  if (point.is_obstacle) {
    lidarDistances[angleDeg] = (uint16_t)(point.distance * 1000.0f);
  } else {
    lidarDistances[angleDeg] = 0;
  }
}

void publishScan(unsigned long current_time) {
  for (int i = 0; i < 360; i++) {
    scan_msg.ranges.data[i] = (lidarDistances[i] == 0)
      ? INFINITY
      : lidarDistances[i] / 1000.0;
  }
  scan_msg.header.stamp.sec = current_time / 1000;
  scan_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;
  rcl_publish(&scan_publisher, &scan_msg, NULL);
}

void setup() {
  Serial.begin(230400);
  delay(2000);
  Serial.println("Starting LiDAR-only node...");

  SerialLidar.begin(230400, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  lidarParser.setAngleUnit(LidarAngleUnit::DEG);
  lidarParser.setDistanceUnit(LidarDistanceUnit::M);
  lidarParser.setResultCallback(onLidarPoint);
  lidarParser.begin();
  Serial.println("LiDAR parser ready.");

  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "lidar_node", "robot1", &support);

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
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = 0.1;
  scan_msg.range_min = 0.05;
  scan_msg.range_max = 12.0;

  rclc_publisher_init_default(
      &scan_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
      "scan"
  );

  Serial.println("Setup complete. Publishing /robot1/scan...");
}

void loop() {
  lidarParser.readData(SerialLidar);

  unsigned long current_time = millis();
  if (current_time - last_publish_time >= PUBLISH_PERIOD_MS) {
    last_publish_time = current_time;
    publishScan(current_time);
  }
}