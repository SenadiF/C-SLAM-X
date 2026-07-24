#include <WiFi.h>
#include <Wire.h>
#include <FastIMU.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <robot_interfaces/msg/encoder.h>
#include <sensor_msgs/msg/imu.h>

#include <rosidl_runtime_c/string_functions.h>
volatile long left_ticks = 0;
volatile long right_ticks = 0;

rcl_publisher_t encoder_publisher;
robot_interfaces__msg__Encoder encoder_msg;


// Constants for unit conversions
#define G_TO_MS2 9.80665f
#define DEG_TO_RAD 0.01745329252f

// WiFi Credentials
const char* WIFI_SSID = "Sena";
const char* WIFI_PASSWORD = "Devanga@123";

// micro ROS Agent
IPAddress AGENT_IP(172, 20, 10, 6);
const uint16_t AGENT_PORT = 8888;

// BMI160
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



// ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;


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
  pinMode(LEFT_ENC_A, INPUT_PULLUP);   // GPIO4 supports internal pull-up
  pinMode(LEFT_ENC_B, INPUT_PULLUP);   // GPIO13 supports internal pull-up
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
void setupMotors() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}
void setup()
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("Starting Robot IMU Node...");

    // Initialize I2C
    Wire.begin();
    Wire.setClock(400000);

    // Initialize micro ROS WiFi transport
    set_microros_wifi_transports(
        "Sena",
        "Devanga@123",
        "172.20.10.6",
        8888
    );
    delay(2000);

    
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

    // Initialize micro ROS core
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "imu_node", "robot1", &support);

    // Create IMU publisher
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu"
    );

    // Initialize BMI160
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
    Serial.println("Motor pins Intilialized");
}

void loop()
{
    
    unsigned long current_time = millis();
    if (current_time - last_publish_time >= PUBLISH_PERIOD_MS) {
        last_publish_time = current_time;

        // Read IMU
        IMU.update();
        IMU.getAccel(&accelData);
        IMU.getGyro(&gyroData);

        // Convert acceleration g units to m/s^2
        float ax = accelData.accelX * G_TO_MS2;
        float ay = accelData.accelY * G_TO_MS2;
        float az = accelData.accelZ * G_TO_MS2;

        // Convert gyro degrees/sec to radians/sec
        float gx = gyroData.gyroX * DEG_TO_RAD;
        float gy = gyroData.gyroY * DEG_TO_RAD;
        float gz = gyroData.gyroZ * DEG_TO_RAD;

        // Populate linear acceleration
        imu_msg.linear_acceleration.x = ax;
        imu_msg.linear_acceleration.y = ay;
        imu_msg.linear_acceleration.z = az;

        // Populate angular velocity
        imu_msg.angular_velocity.x = gx;
        imu_msg.angular_velocity.y = gy;
        imu_msg.angular_velocity.z = gz;

        // Timestamp calculation
        imu_msg.header.stamp.sec = current_time / 1000;
        imu_msg.header.stamp.nanosec = (current_time % 1000) * 1000000;

        
        imu_msg.orientation_covariance[0] = -1;

        // Gyroscope covariance matrix
        imu_msg.angular_velocity_covariance[0] = 0.0004;
        imu_msg.angular_velocity_covariance[4] = 0.0004;
        imu_msg.angular_velocity_covariance[8] = 0.0004;

        // Accelerometer covariance matrix
        imu_msg.linear_acceleration_covariance[0] = 0.04;
        imu_msg.linear_acceleration_covariance[4] = 0.04;
        imu_msg.linear_acceleration_covariance[8] = 0.04;

        // Publish 
        rcl_publish(&imu_publisher, &imu_msg, NULL);
        
       
        encoder_msg.left_ticks = left_ticks;
        encoder_msg.right_ticks = right_ticks;
        rcl_publish(&encoder_publisher, &encoder_msg, NULL);
    }
}
