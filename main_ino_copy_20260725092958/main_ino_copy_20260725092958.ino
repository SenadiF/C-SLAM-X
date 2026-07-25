#include "RosComms.h"
#include "Encoders.h"
#include "ImuSensor.h"
#include "MotorControl.h"

const uint32_t PUBLISH_PERIOD_MS = 10;
unsigned long last_publish_time = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Starting Robot Node...");

  Wire.begin();
  Wire.setClock(400000);

  setupRosCore();
  setupImu();
  setupEncoders();
  setupMotors();

  rclc_subscription_init_default(
      &cmd_vel_subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "cmd_vel"
  );
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
  Serial.println("cmd_vel subscriber ready.");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  updateMotorPID();

  unsigned long current_time = millis();
  if (current_time - last_publish_time >= PUBLISH_PERIOD_MS) {
    last_publish_time = current_time;

    publishImuData(current_time);

    encoder_msg.data.data[0] = left_ticks;
    encoder_msg.data.data[1] = right_ticks;
    rcl_publish(&encoder_publisher, &encoder_msg, NULL);
  }
}