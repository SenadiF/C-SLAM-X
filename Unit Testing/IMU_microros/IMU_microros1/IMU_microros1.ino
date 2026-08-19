#include <WiFi.h>
#include <Wire.h>
#include <FastIMU.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/laser_scan.h>
#include "LidarParserSTL.h"  

#include <sensor_msgs/msg/imu.h>

#include <geometry_msgs/msg/twist.h>   
#include <std_msgs/msg/int32_multi_array.h>

#include <SPI.h>
#include <SD.h>

#define SD_CS_PIN 5
#define LOG_FILENAME "/buffer.bin"

std_msgs__msg__Int32MultiArray encoder_msg;

// Variables for P controller 
unsigned long last_cmd_vel_time=0;
const unsigned long CMD_VEL_TIMEOUT_MS =500;
long prev_left_ticks =0;
long prev_right_ticks =0;
unsigned long last_pid_time=0;
const unsigned long PID_PERIOD_MS=50;

float target_left_speed= 0.0;
float target_right_speed=0.0;

const float KP=5.0;
//dummy value to be removed after calibrating correctly 
const float TICKS_PER_METER = 13313.0;;



#include <rosidl_runtime_c/string_functions.h>
volatile long left_ticks = 0;
volatile long right_ticks = 0;

rcl_publisher_t encoder_publisher;


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
//
//Motor driver
#define LEFT_MOTOR_IN1 2
#define LEFT_MOTOR_IN2 15
#define RIGHT_MOTOR_IN1 27
#define RIGHT_MOTOR_IN2 14

#define LEFT_MOTOR_REVERSED  true
#define RIGHT_MOTOR_REVERSED false


#define LEFT_PWM_CH1 0
#define LEFT_PWM_CH2 1
#define RIGHT_PWM_CH1 2
#define RIGHT_PWM_CH2 3
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

const float WHEEL_BASE_M = 0.099;       
const float MAX_WHEEL_SPEED_MS = 0.6;   

LidarParserSTL lidar;
HardwareSerial LidarSerial(2);
uint16_t lidarDistances[360] = {0};

rcl_publisher_t scan_publisher;         
sensor_msgs__msg__LaserScan scan_msg;  
const uint32_t SCAN_PUBLISH_PERIOD_MS = 100;  
unsigned long last_scan_publish_time = 0;   

// ROS objects
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;


rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;
geometry_msgs__msg__Twist cmd_vel_msg;

//State machine
bool sdReady = false;

enum RobotState { INITIALIZE, NORMAL_MODE, LOCAL_EXPLORATION, SYNC_MODE };
RobotState currentState = INITIALIZE;

unsigned long last_wifi_check = 0;
const unsigned long WIFI_CHECK_PERIOD_MS = 500;

// Log one write / read per cycle 
struct LogRecord {
  long left_ticks;
  long right_ticks;
  float ax, ay, az;
  float gx, gy, gz;
  uint16_t lidar[360];
};

const unsigned long LOG_PERIOD_MS = 100;   // matches the scan publish rate
unsigned long last_log_time = 0;

// SD setup ----
bool setupSd() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD init FAILED — local buffering disabled this session.");
    sdReady = false;
    return false;
  }
  Serial.println("SD card initialized.");
  sdReady = true;
  return true;
}

void logSensorDataToSD() {
  if (!sdReady) return;
  File f = SD.open(LOG_FILENAME, FILE_APPEND);
  if (!f) return;

  LogRecord record;
  record.left_ticks = left_ticks;
  record.right_ticks = right_ticks;
  record.ax = accelData.accelX * G_TO_MS2;
  record.ay = accelData.accelY * G_TO_MS2;
  record.az = accelData.accelZ * G_TO_MS2;
  record.gx = gyroData.gyroX * DEG_TO_RAD;
  record.gy = gyroData.gyroY * DEG_TO_RAD;
  record.gz = gyroData.gyroZ * DEG_TO_RAD;
  memcpy(record.lidar, lidarDistances, sizeof(record.lidar));

  f.write((uint8_t*)&record, sizeof(LogRecord));
  f.close();
}

// Follow the gap implementation
#define SAFE_DISTANCE_MM 400
#define EXPLORE_SPEED_MS 0.30

void followTheGap() {
  int bestStart = 0, bestLen = 0, curStart = -1, curLen = 0;

  for (int i = 0; i < 360; i++) {
    bool clear = (lidarDistances[i] == 0) || (lidarDistances[i] > SAFE_DISTANCE_MM);
    if (clear) {
      if (curStart == -1) curStart = i;
      curLen++;
    } else {
      if (curLen > bestLen) { bestLen = curLen; bestStart = curStart; }
      curStart = -1; curLen = 0;
    }
  }
  if (curLen > bestLen) { bestLen = curLen; bestStart = curStart; }

  if (bestLen == 0) {
    target_left_speed = 0; target_right_speed = 0;
    driveMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, 0, LEFT_MOTOR_REVERSED);
    driveMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, 0, RIGHT_MOTOR_REVERSED);
    return;
  }

  int gapCenter = (bestStart + bestLen / 2) % 360;
  float angleError = gapCenter;
  if (angleError > 180) angleError -= 360;   // shortest turn direction

  target_left_speed  = EXPLORE_SPEED_MS - (angleError * 0.0015);
  target_right_speed = EXPLORE_SPEED_MS + (angleError * 0.0015);

  driveMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, target_left_speed, LEFT_MOTOR_REVERSED);
  driveMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, target_right_speed, RIGHT_MOTOR_REVERSED);
}

// replay buffered records once WiFi is back
void uploadBufferedLogs() {
  if (!sdReady) return;
  File f = SD.open(LOG_FILENAME, FILE_READ);
  if (!f) { Serial.println("No buffered log to upload."); return; }

  Serial.println("Uploading buffered logs...");
  LogRecord record;
  int count = 0;
  unsigned long fakeTime = millis();

  while (f.available() >= (int)sizeof(LogRecord)) {
    f.read((uint8_t*)&record, sizeof(LogRecord));

    // Republish ticks through the SAME publisher as live data
    encoder_msg.data.data[0] = record.left_ticks;
    encoder_msg.data.data[1] = record.right_ticks;
    rcl_publish(&encoder_publisher, &encoder_msg, NULL);

    // Republish IMU through the SAME publisher
    imu_msg.linear_acceleration.x = record.ax;
    imu_msg.linear_acceleration.y = record.ay;
    imu_msg.linear_acceleration.z = record.az;
    imu_msg.angular_velocity.x = record.gx;
    imu_msg.angular_velocity.y = record.gy;
    imu_msg.angular_velocity.z = record.gz;
    imu_msg.header.stamp.sec = 0;
    imu_msg.header.stamp.nanosec = 0;
    rcl_publish(&imu_publisher, &imu_msg, NULL);

    // Republish the LiDAR snapshot through the SAME scan publisher/function
    memcpy(lidarDistances, record.lidar, sizeof(record.lidar));
    publishScan(fakeTime);
    fakeTime += LOG_PERIOD_MS;

    count++;
    delay(10);   
  }
  f.close();
  SD.remove(LOG_FILENAME);
  Serial.print("Replayed records: "); Serial.println(count);
}

//state machine driver
void updateStateMachine() {
  if (millis() - last_wifi_check < WIFI_CHECK_PERIOD_MS) return;
  last_wifi_check = millis();

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);

  switch (currentState) {
    case INITIALIZE:
      if (wifiConnected) currentState = NORMAL_MODE;
      break;

    case NORMAL_MODE:
      if (!wifiConnected) {
        Serial.println("WiFi lost -> LOCAL_EXPLORATION");
        stopMotors();
        currentState = LOCAL_EXPLORATION;
      }
      break;

    case LOCAL_EXPLORATION:
      if (wifiConnected) {
        Serial.println("WiFi back -> SYNC_MODE");
        currentState = SYNC_MODE;
      } else {
        followTheGap();
        if (millis() - last_log_time >= LOG_PERIOD_MS) {
          last_log_time = millis();
          logSensorDataToSD();
        }
      }
      break;

    case SYNC_MODE:
      uploadBufferedLogs();
      currentState = NORMAL_MODE;
      break;
  }
}

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
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      "encoder"
  );
}


void stopMotors() {
  ledcWrite(LEFT_MOTOR_IN1, 0);
  ledcWrite(LEFT_MOTOR_IN2, 0);
  ledcWrite(RIGHT_MOTOR_IN1, 0);
  ledcWrite(RIGHT_MOTOR_IN2, 0);
}

void driveMotor(int pinForward, int pinBackward, float speed, bool reversed) {

  if (reversed)
    speed = -speed;

  int duty = (int)(fabs(speed) / MAX_WHEEL_SPEED_MS * 255.0);
  //int duty = 150;
  duty = constrain(duty, 0, 255);
  const int MIN_DUTY = 100; 
  if (duty > 0 && duty < MIN_DUTY) {
    duty = MIN_DUTY;
  }

  if (speed >= 0) {

    // FORWARD
    ledcWrite(pinForward, 0);
    ledcWrite(pinBackward, duty);

  } 
  else {

    // BACKWARD
    ledcWrite(pinForward, duty);
    ledcWrite(pinBackward, 0);

  }
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_vel_time= millis();
  
  float linear = msg->linear.x;
  float angular = msg->angular.z;

  Serial.print("Linear: ");
  Serial.print(linear);

  Serial.print(" Angular: ");
  Serial.println(angular);

  target_left_speed  = linear - (angular * WHEEL_BASE_M / 2.0);
  target_right_speed = linear + (angular * WHEEL_BASE_M / 2.0);

  Serial.print("target_left_speed: ");
  Serial.println(target_left_speed);

  Serial.print("target_right_speed: ");
  Serial.println(target_right_speed);

  // ---- PID BYPASS: drive directly from cmd_vel, no PID correction ----
  driveMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, target_left_speed, LEFT_MOTOR_REVERSED);
  driveMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, target_right_speed, RIGHT_MOTOR_REVERSED);
}

void setupMotors() {
  ledcAttach(LEFT_MOTOR_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LEFT_MOTOR_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_IN2, PWM_FREQ, PWM_RESOLUTION);

  stopMotors();
}

/* ---- PID DISABLED FOR TESTING ----
void updateMotorPID() {
  if (millis() - last_pid_time < PID_PERIOD_MS) return;
  float dt = (millis() - last_pid_time) / 1000.0;
  last_pid_time = millis();

  // Safety watchdog
  if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
    stopMotors();
    return;
  }

  // Actual speed from encoder ticks this cycle
  float actual_left  = (left_ticks - prev_left_ticks) / TICKS_PER_METER / dt;
  float actual_right = (right_ticks - prev_right_ticks) / TICKS_PER_METER / dt;
  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;

  // proportional correction
  float left_output  = target_left_speed  + KP * (target_left_speed - actual_left) * dt;
  float right_output = target_right_speed + KP * (target_right_speed - actual_right) * dt;

  driveMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, left_output, LEFT_MOTOR_REVERSED);
  driveMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, right_output, RIGHT_MOTOR_REVERSED);
}
*/

void onLidarPoint(const LidarResultData& point, void* ref) {
  int angleDeg = ((int)point.angle) % 360;
  if (angleDeg < 0) angleDeg += 360;
  lidarDistances[angleDeg] = (uint16_t)(point.distance * 1000.0f);
}

void setupLidar() {
  LidarSerial.begin(230400, SERIAL_8N1, 16, 17);
  lidar.setResultCallback(onLidarPoint);
  lidar.setAngleUnit(LidarAngleUnit::DEG);
  lidar.setDistanceUnit(LidarDistanceUnit::M);
  lidar.setLogLevel(LidarLogLevel::OFF);
  lidar.begin();

  rosidl_runtime_c__String__assign(&scan_msg.header.frame_id, "robot1/lidar_link");
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
  scan_msg.range_min = 0.02;
  scan_msg.range_max = 12.0;

  rclc_publisher_init_default(
      &scan_publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
      "scan_raw"
  );
}

void publishScan(unsigned long current_time) {
  for (int i = 0; i < 360; i++) {
    scan_msg.ranges.data[i] = (lidarDistances[i] == 0)
      ? INFINITY
      : lidarDistances[i] / 1000.0;
  }
  scan_msg.header.stamp.sec =0;
  scan_msg.header.stamp.nanosec =0;
  rcl_publish(&scan_publisher, &scan_msg, NULL);
}

void setup()
{
    Serial.begin(115200);

    
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);

    delay(2000);
    Serial.println("Starting Robot IMU Node...");
    

    Wire.begin();
    Serial.println("1");
    Wire.setClock(100000);

    int err = IMU.init(calib, IMU_ADDRESS);

    if (err != 0)
   {
    Serial.print(" IMU initialization failed. Error: ");
    Serial.println(err);

    Serial.println("Continuing without IMU...");
    }
    else
   {
    Serial.println("BMI160 Initialized.");
    }
    Serial.println("2");
    set_microros_wifi_transports(
        "Sena", "Devanga@123", "172.20.10.6", 8888
    );
    delay(2000);
    Serial.println("3");
    delay(2000);
    dacDisable(LEFT_MOTOR_IN1); 
    dacDisable(LEFT_MOTOR_IN2); 

   
    ledcAttach(LEFT_MOTOR_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(LEFT_MOTOR_IN2, PWM_FREQ, PWM_RESOLUTION);
    Serial.println("Starting ...");
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

    allocator = rcl_get_default_allocator();
    Serial.println("r1");
    rclc_support_init(&support, 0, NULL, &allocator);
    Serial.println("r2");
    rclc_node_init_default(&node, "imu_node", "robot1", &support);
    Serial.println("r3");
    rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_raw"
    );

    
    

    setupEncoders();
    Serial.println("Encoders Initialized");

    setupMotors();
    Serial.println("Motor pins Initialized");

    setupLidar();                          
    Serial.println("LiDAR Initialized");   
    encoder_msg.data.data = (int32_t *)malloc(2 * sizeof(int32_t));
    encoder_msg.data.size = 2;
    encoder_msg.data.capacity = 2;
    
    
    setupSd();
    Serial.println("SD Initialized");
    
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
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    // updateMotorPID();   // PID DISABLED FOR TESTING
    lidar.readData(LidarSerial);
    updateStateMachine();

    // Normal mode 
    if (currentState == NORMAL_MODE) {
      if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
        stopMotors();
      }
    }

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

        imu_msg.header.stamp.sec = 0;
        imu_msg.header.stamp.nanosec = 0;

        imu_msg.orientation_covariance[0] = -1;

        imu_msg.angular_velocity_covariance[0] = 0.0004;
        imu_msg.angular_velocity_covariance[4] = 0.0004;
        imu_msg.angular_velocity_covariance[8] = 0.0004;

        imu_msg.linear_acceleration_covariance[0] = 0.04;
        imu_msg.linear_acceleration_covariance[4] = 0.04;
        imu_msg.linear_acceleration_covariance[8] = 0.04;

        rcl_publish(&imu_publisher, &imu_msg, NULL);
        encoder_msg.data.data[0] = left_ticks;
        encoder_msg.data.data[1] = right_ticks;

        rcl_publish(&encoder_publisher, &encoder_msg, NULL);
    }

    if (current_time - last_scan_publish_time >= SCAN_PUBLISH_PERIOD_MS) {
        last_scan_publish_time = current_time;
        if (currentState != LOCAL_EXPLORATION) {  
          publishScan(current_time);
        }
    }
}