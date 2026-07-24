#include <WiFi.h>
#include <Wire.h>
#include <FastIMU.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <robot_interfaces/msg/encoder.h>
#include <sensor_msgs/msg/imu.h>
<<<<<<< HEAD
#include <geometry_msgs/msg/twist.h>   
=======
#include <geometry_msgs/msg/twist.h>   // NEW — needed for cmd_vel message type
>>>>>>> 9808d4a65353e6640fe16f90c7d59ee5127b9e79

#include <rosidl_runtime_c/string_functions.h>
volatile long left_ticks = 0;
volatile long right_ticks = 0;

rcl_publisher_t encoder_publisher;
robot_interfaces__msg__Encoder encoder_msg;

#define G_TO_MS2 9.80665f
#define DEG_TO_RAD 0.01745329252f

const char* WIFI_SSID = "Sena";
const char* WIFI_PASSWORD = "Devanga@123";

IPAddress AGENT_IP(172, 20, 10, 6);
const uint16_t AGENT_PORT = 8888;

#define IMU_ADDRESS 0x69
BMI160 IMU;
calData calib = {0};
AccelData accelData;
GyroData gyroData;

//Encoder
#define LEFT_ENC_A 4
#define LEFT_ENC_B 13
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33

//Motor driver
#define LEFT_MOTOR_IN1 25
#define LEFT_MOTOR_IN2 26
#define RIGHT_MOTOR_IN1 27
#define RIGHT_MOTOR_IN2 14

#define LEFT_MOTOR_REVERSED  false
#define RIGHT_MOTOR_REVERSED false


#define LEFT_PWM_CH1 0
#define LEFT_PWM_CH2 1
#define RIGHT_PWM_CH1 2
#define RIGHT_PWM_CH2 3
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

const float WHEEL_BASE_M = 0.099;       
const float MAX_WHEEL_SPEED_MS = 0.3;   

// ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

// NEW — for receiving cmd_vel
rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_vel_msg;

rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_vel_msg;

const uint32_t PUBLISH_PERIOD_MS = 10;
unsigned long last_publish_time = 0;

void IRAM_ATTR leftEncoderISR() {
  bool a = digitalRead(LEFT_ENC_A);
  bool b = digitalRead(LEFT_ENC_B);
  left_ticks += (a == b) ? 1 : -1;
}

void IRAM_ATTR rightEncoderISR() {
  bool a = digitalRead(RIGHT_ENC_A);
  bool b = digitalRead(RIGHT_ENC_B);
  right_ticks += (a == b) ? 1 : -1;
}

void setupEncoders() {
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);
  rclc_publisher_init_default(
      &encoder_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(robot_interfaces, msg, Encoder),
      "encoder"
  );
}


void stopMotors() {
  ledcWrite(LEFT_PWM_CH1, 0);  ledcWrite(LEFT_PWM_CH2, 0);
  ledcWrite(RIGHT_PWM_CH1, 0); ledcWrite(RIGHT_PWM_CH2, 0);
}

void driveMotor(int chForward, int chBackward, float speed, bool reversed) {
  if (reversed) speed = -speed;
  int duty = (int)(fabs(speed) / MAX_WHEEL_SPEED_MS * 255.0);
  duty = constrain(duty, 0, 255);
  if (speed >= 0) { ledcWrite(chForward, duty); ledcWrite(chBackward, 0); }
  else            { ledcWrite(chForward, 0);    ledcWrite(chBackward, duty); }
}


void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  float left_speed  = linear - (angular * WHEEL_BASE_M / 2.0);
  float right_speed = linear + (angular * WHEEL_BASE_M / 2.0);

  driveMotor(LEFT_PWM_CH1, LEFT_PWM_CH2, left_speed, LEFT_MOTOR_REVERSED);
  driveMotor(RIGHT_PWM_CH1, RIGHT_PWM_CH2, right_speed, RIGHT_MOTOR_REVERSED);
}

void setupMotors() {
  ledcSetup(LEFT_PWM_CH1, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(LEFT_MOTOR_IN1, LEFT_PWM_CH1);
  ledcSetup(LEFT_PWM_CH2, PWM_FREQ, PWM_RESOLUTION);  ledcAttachPin(LEFT_MOTOR_IN2, LEFT_PWM_CH2);
  ledcSetup(RIGHT_PWM_CH1, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(RIGHT_MOTOR_IN1, RIGHT_PWM_CH1);
  ledcSetup(RIGHT_PWM_CH2, PWM_FREQ, PWM_RESOLUTION); ledcAttachPin(RIGHT_MOTOR_IN2, RIGHT_PWM_CH2);
  stopMotors();
}

void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("Starting Robot IMU Node...");

    Wire.begin();
    Wire.setClock(400000);

    set_microros_wifi_transports(
        "Sena",
        "Devanga@123",
        "172.20.10.6",
        8888
    );
    delay(2000);

    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "imu_node", "robot1", &support);

    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"
    );

    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0)
    {
        Serial.print("IMU initialization failed: ");
        Serial.println(err);
        while (true);
    }
    Serial.println("BMI160 Initialized.");

    setupEncoders();
    Serial.println("Encoders Initialized");

    setupMotors();
    Serial.println("Motor pins Initialized");

    
    rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    Serial.println("cmd_vel subscriber ready.");
}

void loop()
{
    // Spin the executor to handle incoming messages
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    unsigned long current_time = millis();
    if (current_time - last_publish_time >= PUBLISH_PERIOD_MS) {
        last_publish_time = current_time;

        IMU.update();
        IMU.getAccel(&accelData);
        IMU.getGyro(&gyroData);

        float ax = accelData.accelX * G_TO_MS2;
        float ay = accelData.accelY * G_TO_MS2;
        float az = accelData.accelZ * G_TO_MS2;

        float gx = gyroData.gyroX * DEG_TO_RAD;
        float gy = gyroData.gyroY * DEG_TO_RAD;
        float gz = gyroData.gyroZ * DEG_TO_RAD;

        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;

        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;

        imu_msg.header.stamp.sec = current_time / 1000;
        imu_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;

        imu_msg.orientation_covariance[0] = -1;

        imu_msg.angular_velocity_covariance[0] = 0.0004;
        imu_msg.angular_velocity_covariance[4] = 0.0004;
        imu_msg.angular_velocity_covariance[8] = 0.0004;

        imu_msg.linear_acceleration_covariance[0] = 0.04;
        imu_msg.linear_acceleration_covariance[4] = 0.04;
        imu_msg.linear_acceleration_covariance[8] = 0.04;

        rcl_publish(&imu_publisher, &imu_msg, NULL);

        encoder_msg.left_ticks = left_ticks;
        encoder_msg.right_ticks = right_ticks;
        rcl_publish(&encoder_publisher, &encoder_msg, NULL);
    }
}