#include <WiFi.h>
#include <Wire.h>
#include <FastIMU.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// Which robot this board is - the only lines that differ between the two firmwares.
#define ROBOT_NAME "robot2"
#define CLIENT_KEY 0x00000002
#include <sensor_msgs/msg/laser_scan.h>
#include "LidarParserSTL.h"  

#include <sensor_msgs/msg/imu.h>

#include <geometry_msgs/msg/twist.h>   
#include <std_msgs/msg/int32_multi_array.h>

std_msgs__msg__Int32MultiArray encoder_msg;

// Closed-loop wheel speed control: feedforward + PI on encoder speed.
// Open-loop PWM gives the same power in the air and on the floor, so the
// robot drives slower than commanded under load. This measures real wheel
// speed and raises/lowers the PWM until it matches the command.
// Set ENABLE_SPEED_PID to 0 to go back to the old open-loop behaviour.
#define ENABLE_SPEED_PID 1
#define PID_DEBUG 1   // 1 = print target/measured speed on Serial for tuning

unsigned long last_cmd_vel_time=0;
const unsigned long CMD_VEL_TIMEOUT_MS =500;
unsigned long last_pid_time=0;
const unsigned long PID_PERIOD_MS=50;

float target_left_speed= 0.0;
float target_right_speed=0.0;

// Starting guess for encoder direction (matches wheel_odometry_node's
// left/right_encoder_sign). If a wheel is measured rolling the opposite way
// to its command, the controller flips its sign automatically, so a wrong
// guess here self-corrects within about 0.3 s.
const float LEFT_ENCODER_SIGN = -1.0;
const float RIGHT_ENCODER_SIGN = 1.0;

// Per-wheel controller state. closed_loop drops to false if the encoder
// never responds, and that wheel falls back to plain open-loop PWM.
struct WheelCtrl {
  const char *label;
  float sign;
  float integral;
  float measured;
  long prev_ticks;
  unsigned long bad_since;
  bool closed_loop;
};
WheelCtrl left_wheel  = {"L", LEFT_ENCODER_SIGN,  0.0, 0.0, 0, 0, true};
WheelCtrl right_wheel = {"R", RIGHT_ENCODER_SIGN, 0.0, 0.0, 0, 0, true};

const float KP = 300.0;            // PWM duty per 1 m/s of speed error
const float KI = 600.0;            // PWM duty per 1 m/s*s of accumulated error
const float INTEGRAL_LIMIT = 0.3;  // caps the KI term at about 180 duty
const float TICKS_PER_METER = 13313.0;

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

// Gyro zero-rate offset (deg/s), measured at boot. Uncorrected, robot2's
// gyro read +0.0044 rad/s standing still; the EKF trusts the gyro for yaw,
// so the heading drifted ~15 deg/min and SLAM's scans rotated (ghosting).
float gyro_bias[3] = {0.0, 0.0, 0.0};

// Average the gyro while the robot is still. Keep it untouched for ~2 s
// after power-on.
void calibrateGyroBias() {
  const int samples = 400;
  float sum[3] = {0.0, 0.0, 0.0};
  GyroData g;
  for (int i = 0; i < samples; i++) {
    IMU.update();
    IMU.getGyro(&g);
    sum[0] += g.gyroX;
    sum[1] += g.gyroY;
    sum[2] += g.gyroZ;
    delay(5);
  }
  for (int k = 0; k < 3; k++) {
    gyro_bias[k] = sum[k] / samples;
  }
  Serial.printf("Gyro bias (deg/s): %.3f %.3f %.3f\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
}
AccelData accelData;
GyroData gyroData;

//Encoder
#define LEFT_ENC_A 4
#define LEFT_ENC_B 13
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 35

//Motor driver
#define LEFT_MOTOR_IN1 2
#define LEFT_MOTOR_IN2 15
#define RIGHT_MOTOR_IN1 14
#define RIGHT_MOTOR_IN2 27

#define LEFT_MOTOR_REVERSED  false
#define RIGHT_MOTOR_REVERSED true

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

const float WHEEL_BASE_M = 0.099;       
const float MAX_WHEEL_SPEED_MS = 0.6;   

LidarParserSTL lidar;
HardwareSerial LidarSerial(2);
uint16_t lidarDistances[360] = {0};
// millis() when each angle was last measured. A reading older than
// LIDAR_MAX_AGE_MS is published as "no return": it was taken from where the
// robot used to be, and SLAM would map it as a ghost wall. The LD19 sweeps
// every ~100 ms, so 250 ms keeps a full rotation even if one sweep is late.
unsigned long lidarStamps[360] = {0};
const unsigned long LIDAR_MAX_AGE_MS = 250;

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

const uint32_t PUBLISH_PERIOD_MS = 20;  // 50 Hz IMU/encoder - plenty for the 30 Hz EKF, half the Wi-Fi load
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

}

void stopMotors() {
  ledcWrite(LEFT_MOTOR_IN1, 0);
  ledcWrite(LEFT_MOTOR_IN2, 0);
  ledcWrite(RIGHT_MOTOR_IN1, 0);
  ledcWrite(RIGHT_MOTOR_IN2, 0);
}

// Lowest PWM that still turns the wheels on the floor. Turning on the spot
// (wheels in opposite directions) loads the motors much more than driving
// straight, so it gets a higher floor - at 150 the robot barely rotated.
const int MIN_DUTY = 180;
const int MIN_TURN_DUTY = 220;

// duty is signed: positive = forward, range -255..255.
void driveMotorDuty(int pinForward, int pinBackward, float duty, bool reversed, int min_duty) {
  if (reversed)
    duty = -duty;

  int d = (int)fabs(duty);
  d = constrain(d, 0, 255);
  if (d > 0 && d < min_duty) {
    d = min_duty;
  }

  if (duty >= 0) {
    ledcWrite(pinForward, 0);
    ledcWrite(pinBackward, d);
  } 
  else {
    ledcWrite(pinForward, d);
    ledcWrite(pinBackward, 0);
  }
}

void driveMotor(int pinForward, int pinBackward, float speed, bool reversed) {
  driveMotorDuty(pinForward, pinBackward, speed / MAX_WHEEL_SPEED_MS * 255.0, reversed, MIN_DUTY);
}

float wheelStep(WheelCtrl *w, float target, long ticks, float dt, unsigned long now) {
  long dticks = ticks - w->prev_ticks;
  float measured = w->sign * dticks / TICKS_PER_METER / dt;
  w->prev_ticks = ticks;
  w->measured = measured;

  float feedforward = target / MAX_WHEEL_SPEED_MS * 255.0;
  if (target == 0.0) {
    w->integral = 0.0;
    w->bad_since = 0;
    return 0.0;
  }
  if (!w->closed_loop) {
    if (dticks == 0) {
      return feedforward;
    }
    // Ticks are back (e.g. it was only stalled) - resume speed control.
    w->closed_loop = true;
    w->bad_since = 0;
    Serial.printf("[speed] %s encoder responding again - back to PI\n", w->label);
  }

  bool wrong_direction = (measured * target < 0.0) && fabs(measured) > 0.05;
  // Only zero ticks counts as a dead encoder. A slow or stalled wheel under
  // load still ticks a little, and must stay closed-loop so the PI can
  // raise its PWM - giving up on it left it stuck at the minimum duty.
  bool not_moving = (dticks == 0);
  if (wrong_direction || not_moving) {
    if (w->bad_since == 0) {
      w->bad_since = now;
    }
  } else {
    w->bad_since = 0;
  }

  if (wrong_direction && now - w->bad_since > 300) {
    w->sign = -w->sign;
    w->integral = 0.0;
    w->bad_since = 0;
    Serial.printf("[speed] %s encoder sign was reversed - flipped to %.0f\n", w->label, w->sign);
    return feedforward;
  }
  if (not_moving && now - w->bad_since > 1500) {
    w->closed_loop = false;
    w->integral = 0.0;
    Serial.printf("[speed] %s encoder not responding - using open-loop PWM for this wheel\n", w->label);
    return feedforward;
  }

  float error = target - measured;
  w->integral = constrain(w->integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  return feedforward + KP * error + KI * w->integral;
}

void updateSpeedControl() {
  unsigned long now = millis();
  if (now - last_pid_time < PID_PERIOD_MS) {
    return;
  }
  float dt = (now - last_pid_time) / 1000.0;
  last_pid_time = now;

  long l = left_ticks;
  long r = right_ticks;

  if (now - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
    target_left_speed = 0.0;
    target_right_speed = 0.0;
  }

  float left_duty = wheelStep(&left_wheel, target_left_speed, l, dt, now);
  float right_duty = wheelStep(&right_wheel, target_right_speed, r, dt, now);

  if (target_left_speed == 0.0 && target_right_speed == 0.0) {
    stopMotors();
  } else {
    int min_duty = (target_left_speed * target_right_speed < 0.0) ? MIN_TURN_DUTY : MIN_DUTY;
    driveMotorDuty(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, left_duty, LEFT_MOTOR_REVERSED, min_duty);
    driveMotorDuty(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, right_duty, RIGHT_MOTOR_REVERSED, min_duty);
  }

#if PID_DEBUG
  static unsigned long last_debug = 0;
  if (now - last_debug > 500) {
    last_debug = now;
    Serial.printf("L tgt %.2f meas %.2f duty %.0f %s | R tgt %.2f meas %.2f duty %.0f %s | ticks %ld %ld\n",
                  target_left_speed, left_wheel.measured, constrain(left_duty, -255.0, 255.0),
                  left_wheel.closed_loop ? "PI" : "OPEN",
                  target_right_speed, right_wheel.measured, constrain(right_duty, -255.0, 255.0),
                  right_wheel.closed_loop ? "PI" : "OPEN",
                  l, r);
  }
#endif
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

#if !ENABLE_SPEED_PID
  driveMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, target_left_speed, LEFT_MOTOR_REVERSED);
  driveMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, target_right_speed, RIGHT_MOTOR_REVERSED);
#endif
}

void setupMotors() {
  ledcAttach(LEFT_MOTOR_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LEFT_MOTOR_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_IN2, PWM_FREQ, PWM_RESOLUTION);
  stopMotors();
}

// The LDROBOT STL library reports angles clockwise (see LidarParserSTL.h),
// but ROS LaserScan is counter-clockwise. Without converting, every scan
// reaches ROS mirrored left-right: SLAM smears the map on every turn and
// gap-following/obstacle avoidance steer the wrong way. Set to 0 only if
// the LiDAR is mounted upside down.
#define LIDAR_CLOCKWISE 1

void onLidarPoint(const LidarResultData& point, void* ref) {
  int angleDeg = ((int)point.angle) % 360;
  if (angleDeg < 0) angleDeg += 360;
#if LIDAR_CLOCKWISE
  angleDeg = (360 - angleDeg) % 360;
#endif
  lidarDistances[angleDeg] = (uint16_t)(point.distance * 1000.0f);
  lidarStamps[angleDeg] = millis();
}

void setupLidar() {
  // The LD19 streams ~23 KB/s; the default 256-byte buffer overflows (and
  // loses points) whenever the loop blocks on Wi-Fi for more than ~10 ms.
  LidarSerial.setRxBufferSize(4096);
  LidarSerial.begin(230400, SERIAL_8N1, 16, 17);
  lidar.setResultCallback(onLidarPoint);
  lidar.setAngleUnit(LidarAngleUnit::DEG);
  lidar.setDistanceUnit(LidarDistanceUnit::M);
  lidar.setLogLevel(LidarLogLevel::OFF);
  lidar.begin();

  rosidl_runtime_c__String__assign(&scan_msg.header.frame_id, ROBOT_NAME "/lidar_link");
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

}

// readData() parses only ONE 12-point packet per call, and the LD19 sends
// ~375 packets/s. Called once per loop() it kept ~2% of the points: the
// rest overflowed the serial buffer, so scans were mostly seconds-old
// readings (the ghost walls) or, once those expired, nearly empty.
// Parse every complete packet waiting in the buffer. A packet is 47 bytes;
// readData() drops a frame if bytes run out mid-way, so stop before that.
const int LIDAR_PACKET_BYTES = 47;
const int LIDAR_MAX_PACKETS_PER_DRAIN = 64;  // ~170 ms of data, bounds the time spent here

void drainLidar() {
  for (int i = 0; i < LIDAR_MAX_PACKETS_PER_DRAIN && LidarSerial.available() >= LIDAR_PACKET_BYTES; i++) {
    lidar.readData(LidarSerial);
  }
}

void publishScan(unsigned long current_time) {
  drainLidar();  // freshest points into this scan
  unsigned long now = millis();
  for (int i = 0; i < 360; i++) {
    // 0 = no reading; the STL also reports failed returns as ~65 m.
    bool stale = now - lidarStamps[i] > LIDAR_MAX_AGE_MS;
    scan_msg.ranges.data[i] = (stale || lidarDistances[i] == 0 || lidarDistances[i] > 12000)
      ? INFINITY
      : lidarDistances[i] / 1000.0;
  }
  scan_msg.header.stamp.sec = 0;
  scan_msg.header.stamp.nanosec = 0;
  rcl_publish(&scan_publisher, &scan_msg, NULL);
}

// ---------------------------------------------------------------------
// micro-ROS connection handling. If the agent crashes or Wi-Fi drops, the
// board stops its motors, waits for the agent to come back, and re-creates
// its node/topics on its own - no power-cycling needed.
// ---------------------------------------------------------------------
enum AgentState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
AgentState agent_state = WAITING_AGENT;
unsigned long last_agent_check = 0;

#define RCCHECK_BOOL(fn) { rcl_ret_t rc = (fn); if (rc != RCL_RET_OK) { Serial.printf("[ros] step failed at line %d (rc=%d)\n", __LINE__, (int)rc); return false; } }

bool createEntities() {
  allocator = rcl_get_default_allocator();

  // Each board needs a distinct XRCE-DDS client key, or the agent can
  // treat both robots as one session and send commands to the wrong robot.
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK_BOOL(rcl_init_options_init(&init_options, allocator));
  rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
  RCCHECK_BOOL(rmw_uros_options_set_client_key(CLIENT_KEY, rmw_options));
  RCCHECK_BOOL(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  rcl_init_options_fini(&init_options);

  RCCHECK_BOOL(rclc_node_init_default(&node, "imu_node", ROBOT_NAME, &support));
  // IMU and encoder go best-effort: they are small and frequent, and on the
  // reliable stream they got interleaved with the fragments of the ~1.5 KB
  // LaserScan, corrupting it (agent "deserialization error ... WRITE_DATA").
  // Now the scan is the only thing on the reliable (fragmenting) stream.
  RCCHECK_BOOL(rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu_raw"));
  RCCHECK_BOOL(rclc_publisher_init_best_effort(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "encoder"));
  RCCHECK_BOOL(rclc_publisher_init_default(&scan_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan_raw"));
  // A reliable publish waits for the agent's ACK for up to
  // RMW_UXRCE_PUBLISH_RELIABLE_TIMEOUT (1000 ms). On a lossy Wi-Fi link that
  // stalled the whole loop for 0.7-2 s per scan, so IMU/encoders/scans all
  // dropped to ~1 Hz and odometry and scans no longer lined up (ghosting).
  // Wait at most 20 ms; unacknowledged fragments are resent on later spins.
  RCCHECK_BOOL(rmw_uros_set_publisher_session_timeout(rcl_publisher_get_rmw_handle(&scan_publisher), 20));
  RCCHECK_BOOL(rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));

  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK_BOOL(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK_BOOL(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  return true;
}

void destroyEntities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&imu_publisher, &node);
  rcl_publisher_fini(&encoder_publisher, &node);
  rcl_publisher_fini(&scan_publisher, &node);
  rcl_subscription_fini(&cmd_vel_subscriber, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void haltRobot() {
  target_left_speed = 0.0;
  target_right_speed = 0.0;
  stopMotors();
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
    Serial.println("=== THIS BOARD IS: " ROBOT_NAME " ===");

    Wire.begin();
    Wire.setClock(100000);

    int err = IMU.init(calib, IMU_ADDRESS);
    if (err != 0) {
        Serial.print(" IMU initialization failed. Error: ");
        Serial.println(err);
        Serial.println("Continuing without IMU...");
    } else {
        Serial.println("BMI160 Initialized.");
        calibrateGyroBias();
    }

    Serial.println("Connecting to Wi-Fi...");
    set_microros_wifi_transports(
        "Sena", "Devanga@123", "172.20.10.6", 8888
    );
    // Wi-Fi power save (modem sleep, on by default) holds incoming packets
    // until the next beacon: pings to this board took 6-209 ms (hotspot:
    // 4 ms). The agent's ACKs arrived late, the reliable scan stream backed
    // up, and every loop stalled ~1 s - sensors at 1 Hz, ghosted maps.
    WiFi.setSleep(false);
    Serial.println("Wi-Fi connected.");

    dacDisable(LEFT_MOTOR_IN1); 
    dacDisable(LEFT_MOTOR_IN2); 

    ledcAttach(LEFT_MOTOR_IN1, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(LEFT_MOTOR_IN2, PWM_FREQ, PWM_RESOLUTION);

    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, ROBOT_NAME "/imu_link");
    encoder_msg.data.data = (int32_t *)malloc(2 * sizeof(int32_t));
    encoder_msg.data.size = 2;
    encoder_msg.data.capacity = 2;

    setupEncoders();
    setupMotors();
    setupLidar();
    Serial.println("Hardware ready. Waiting for micro-ROS agent...");
}

void publishSensors(unsigned long current_time)
{
    if (current_time - last_publish_time >= PUBLISH_PERIOD_MS) {
        last_publish_time = current_time;

        IMU.update();
        IMU.getAccel(&accelData);
        IMU.getGyro(&gyroData);

        imu_msg.linear_acceleration.x = accelData.accelX * G_TO_MS2;
        imu_msg.linear_acceleration.y = accelData.accelY * G_TO_MS2;
        imu_msg.linear_acceleration.z = accelData.accelZ * G_TO_MS2;

        imu_msg.angular_velocity.x = (gyroData.gyroX - gyro_bias[0]) * DEG_TO_RAD;
        imu_msg.angular_velocity.y = (gyroData.gyroY - gyro_bias[1]) * DEG_TO_RAD;
        imu_msg.angular_velocity.z = (gyroData.gyroZ - gyro_bias[2]) * DEG_TO_RAD;

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
        publishScan(current_time);
    }
}

void loop()
{
    drainLidar();
    unsigned long now = millis();

    switch (agent_state) {
      case WAITING_AGENT:
        haltRobot();
        if (now - last_agent_check > 500) {
          last_agent_check = now;
          if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            agent_state = AGENT_AVAILABLE;
          }
        }
        break;

      case AGENT_AVAILABLE:
        if (createEntities()) {
          Serial.println("[ros] connected to agent - topics created.");
          last_agent_check = now;
          agent_state = AGENT_CONNECTED;
        } else {
          Serial.println("[ros] setup failed - retrying.");
          destroyEntities();
          agent_state = WAITING_AGENT;
        }
        break;

      case AGENT_CONNECTED:
        if (now - last_agent_check > 1000) {
          last_agent_check = now;
          if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
            agent_state = AGENT_DISCONNECTED;
            break;
          }
        }
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // Watchdog: stop if no cmd_vel for CMD_VEL_TIMEOUT_MS.
        if (now - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
          stopMotors();
        }
#if ENABLE_SPEED_PID
        updateSpeedControl();
#endif
        publishSensors(now);
        break;

      case AGENT_DISCONNECTED:
        Serial.println("[ros] lost the agent - motors stopped, reconnecting...");
        haltRobot();
        destroyEntities();
        agent_state = WAITING_AGENT;
        break;
    }
}
