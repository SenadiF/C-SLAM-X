#include <micro_ros_arduino.h>
#include <WiFi.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>

#include <std_msgs/msg/int32.h>

char ssid[] = "Sena";
char password[] = "Devanga@123";
char agent_ip[] = "172.20.10.6";

rcl_publisher_t publisher;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_support_t support;

std_msgs__msg__Int32 msg;

#define LED_PIN 2

void error_loop()
{
  while (true)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(200);
  }
}

void setup()
{
  pinMode(LED_PIN, OUTPUT);

  Serial.begin(115200);
  delay(2000);

  Serial.println("Connecting WiFi...");

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi Connected");
  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  set_microros_wifi_transports(
      ssid,
      password,
      agent_ip,
      8888);

  delay(2000);

  allocator = rcl_get_default_allocator();

  rcl_ret_t rc;

  rc = rclc_support_init(
      &support,
      0,
      NULL,
      &allocator);

  Serial.print("Support rc = ");
  Serial.println(rc);

  if(rc != RCL_RET_OK)
      error_loop();

  rc = rclc_node_init_default(
      &node,
      "esp32_node",
      "",
      &support);

  Serial.print("Node rc = ");
  Serial.println(rc);

  if(rc != RCL_RET_OK)
      error_loop();

  rc = rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "esp32_counter");

  Serial.print("Publisher rc = ");
  Serial.println(rc);

  if(rc != RCL_RET_OK)
      error_loop();

  msg.data = 0;

  Serial.println("Setup Complete");
}

void loop()
{
  rcl_ret_t rc;

  rc = rcl_publish(
      &publisher,
      &msg,
      NULL);

  Serial.print("Publish rc = ");
  Serial.print(rc);
  Serial.print("   value = ");
  Serial.println(msg.data);

  msg.data++;

  delay(1000);
}