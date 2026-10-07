#include <WiFi.h>
#include <lwip/sockets.h>
#include <Wire.h>
#include <FastIMU.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/util/time.h>
// Fixes micro-ROS's clock on this core (it only advanced in whole seconds,
// holding loop() and every topic at ~1 Hz). Install:
// firmware_libs/uros_clock_fix/extras/README.md
#include <uros_clock_fix.h>

// Which robot this board is - the only lines that differ between the two firmwares.
#define ROBOT_NAME "robot1"
#define CLIENT_KEY 0x00000001
#include <sensor_msgs/msg/laser_scan.h>
#include "LidarParserSTL.h"  

#include <sensor_msgs/msg/imu.h>

#include <geometry_msgs/msg/twist.h>   
#include <std_msgs/msg/int32_multi_array.h>

std_msgs__msg__Int32MultiArray encoder_msg;

// Loop timing diagnostics. The encoder message is [left, right, then the
// longest time (ms) each step took since the previous encoder message]:
// loop, drainLidar, agent ping, executor spin, speed PID, scan publish;
// then the left and right encoder interrupt counts since boot; then the
// micro-ROS transport read stats (largest requested timeout ms, longest
// read ms, count of "wait forever" reads since boot); then uxr_millis()
// and millis() (low 32 bits) to compare micro-ROS's clock with Arduino's.
// wheel_odometry_node only reads [0] and [1]; scripts/sensor_health.py
// prints the rest. Used to find what stalls the loop (sensors at ~1 Hz).
enum { T_LOOP, T_DRAIN, T_PING, T_SPIN, T_PID, T_SCAN, T_COUNT };
uint32_t t_max_ms[T_COUNT] = {0};
const int ENCODER_MSG_LEN = 2 + T_COUNT + 2 + 3 + 2;
#define TIME_MAX(slot, stmt) { uint32_t _t0 = micros(); stmt; \
  uint32_t _d = (micros() - _t0) / 1000; if (_d > t_max_ms[slot]) t_max_ms[slot] = _d; }

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

// The 4G dongle plugged into the laptop by USB: robots reach it over Wi-Fi,
// the dongle reaches the laptop over the cable - one wireless hop instead of
// two via the phone hotspot (which lost 31-82% of pings to the robots).
const char* WIFI_SSID = "Sena";
const char* WIFI_PASSWORD = "25112003";
// The phone hotspot is also called "Sena" (different password), so join the
// dongle by its radio address only. 0 channel = scan all.
const uint8_t WIFI_BSSID[6] = {0xE4, 0x7D, 0xEB, 0x82, 0x54, 0x79};

// The laptop's address on the dongle's network (the agent runs there).
// Static on the laptop's USB connection ("Wired connection 2", set with
// nmcli), outside the dongle's DHCP range (.100+): with DHCP the dongle
// reshuffled addresses and once handed the agent's address to a robot.
IPAddress AGENT_IP(192, 168, 0, 50);
const char *AGENT_IP_STR = "192.168.0.50";
const uint16_t AGENT_PORT = 8888;

// ---------------------------------------------------------------------
// micro-ROS UDP transport on plain lwIP sockets, replacing the library's
// WiFiUDP one. With the library transport the executor spin took ~970 ms
// every loop although it asks for 10 ms; select() here waits exactly the
// requested time, and the read stats below (sent in the encoder message)
// show what timeouts micro-ROS actually requests.
// ---------------------------------------------------------------------
static int uros_sock = -1;
static struct sockaddr_in uros_agent_addr;
int32_t read_req_max_ms = 0;   // largest timeout micro-ROS asked a read to wait
int32_t read_took_max_ms = 0;  // longest a read actually took
int32_t read_inf_count = 0;    // reads asked to wait forever (timeout < 0)

bool uros_udp_open(struct uxrCustomTransport *transport) {
  uros_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (uros_sock < 0) {
    return false;
  }
  struct sockaddr_in local;
  memset(&local, 0, sizeof(local));
  local.sin_family = AF_INET;
  local.sin_port = htons(0);
  local.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(uros_sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
    close(uros_sock);
    uros_sock = -1;
    return false;
  }
  memset(&uros_agent_addr, 0, sizeof(uros_agent_addr));
  uros_agent_addr.sin_family = AF_INET;
  uros_agent_addr.sin_port = htons(AGENT_PORT);
  uros_agent_addr.sin_addr.s_addr = inet_addr(AGENT_IP_STR);
  return true;
}

bool uros_udp_close(struct uxrCustomTransport *transport) {
  if (uros_sock >= 0) {
    close(uros_sock);
  }
  uros_sock = -1;
  return true;
}

size_t uros_udp_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode) {
  int sent = sendto(uros_sock, buf, len, 0, (struct sockaddr *)&uros_agent_addr, sizeof(uros_agent_addr));
  if (sent < 0) {
    *errcode = 1;
    return 0;
  }
  return sent;
}

size_t uros_udp_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode) {
  uint32_t t0 = millis();
  if (timeout < 0) {
    // "Wait forever" - would freeze loop(); cap it and count it.
    read_inf_count++;
    timeout = 100;
  }
  if (timeout > read_req_max_ms) {
    read_req_max_ms = timeout;
  }

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(uros_sock, &fds);
  struct timeval tv;
  tv.tv_sec = timeout / 1000;
  tv.tv_usec = (timeout % 1000) * 1000;

  size_t n = 0;
  if (select(uros_sock + 1, &fds, NULL, NULL, &tv) > 0) {
    int got = recv(uros_sock, buf, len, 0);
    if (got > 0) {
      n = got;
    }
  }

  int32_t took = millis() - t0;
  if (took > read_took_max_ms) {
    read_took_max_ms = took;
  }
  return n;
}

#define IMU_ADDRESS 0x69
BMI160 IMU;
calData calib = {0};

// Gyro zero-rate offset (deg/s), measured at boot. Uncorrected, robot2's
// gyro read +0.0044 rad/s standing still; the EKF trusts the gyro for yaw,
// so the heading drifted ~15 deg/min and SLAM's scans rotated (ghosting).
float gyro_bias[3] = {0.0, 0.0, 0.0};

// Boot calibration. After power-on/reset there is GYRO_CALIB_SETTLE_MS to
// put the robot down and let go, then the gyro is averaged for
// GYRO_CALIB_MS. If the yaw rate varied more than GYRO_CALIB_MAX_SPREAD_DPS
// in that window the robot was moved, and the window is repeated (up to
// GYRO_CALIB_ATTEMPTS). Total: about 8 s untouched. Whatever offset remains
// is corrected while driving by updateHeading()'s stationary tracking.
const unsigned long GYRO_CALIB_SETTLE_MS = 3000;
const unsigned long GYRO_CALIB_MS = 5000;
const float GYRO_CALIB_MAX_SPREAD_DPS = 1.0;
const int GYRO_CALIB_ATTEMPTS = 5;

void calibrateGyroBias() {
  Serial.printf("Gyro calibration: keep the robot still. Starting in %lu s...\n",
                GYRO_CALIB_SETTLE_MS / 1000);
  delay(GYRO_CALIB_SETTLE_MS);

  for (int attempt = 1; attempt <= GYRO_CALIB_ATTEMPTS; attempt++) {
    double sum[3] = {0.0, 0.0, 0.0};
    float z_min = 1e9, z_max = -1e9;
    long samples = 0;
    GyroData g;
    unsigned long start = millis();
    while (millis() - start < GYRO_CALIB_MS) {
      IMU.update();
      IMU.getGyro(&g);
      sum[0] += g.gyroX;
      sum[1] += g.gyroY;
      sum[2] += g.gyroZ;
      z_min = min(z_min, g.gyroZ);
      z_max = max(z_max, g.gyroZ);
      samples++;
      delay(5);
    }
    for (int k = 0; k < 3; k++) {
      gyro_bias[k] = sum[k] / samples;
    }
    float spread = z_max - z_min;
    if (spread <= GYRO_CALIB_MAX_SPREAD_DPS) {
      Serial.printf("Gyro bias (deg/s): %.3f %.3f %.3f  (yaw spread %.2f, attempt %d)\n",
                    gyro_bias[0], gyro_bias[1], gyro_bias[2], spread, attempt);
      return;
    }
    Serial.printf("Gyro calibration: robot moved (yaw spread %.2f deg/s) - keep it still, retrying (%d/%d)\n",
                  spread, attempt, GYRO_CALIB_ATTEMPTS);
  }
  Serial.printf("Gyro calibration: never still - using last average %.3f deg/s; corrected later while parked\n",
                gyro_bias[2]);
}
AccelData accelData;
GyroData gyroData;

// ---------------------------------------------------------------------
// Heading integrated on the ESP32, published as the IMU orientation (yaw).
// The EKF used to integrate the gyro *rate* from whatever IMU messages made
// it over Wi-Fi, so dropped messages and outages lost heading, and a gyro
// offset left after the boot calibration (e.g. the robot was touched) made
// a parked robot's heading creep - both showed as rotated/ghosted walls.
// Here every reading is integrated locally, and while both encoders report
// no movement the robot cannot be turning: the reading is then treated as
// gyro offset (tracked continuously, also covering warm-up drift) and
// nothing is integrated.
// ---------------------------------------------------------------------
const unsigned long HEADING_PERIOD_US = 10000;  // ~100 Hz when loop() allows
const unsigned long STILL_MS = 500;             // no encoder ticks this long = stationary
const float STILL_MAX_RATE_DPS = 3.0;           // larger readings are real motion, not offset
const float BIAS_TRACK_ALPHA = 0.01;            // per sample, ~1 s time constant at 100 Hz

float heading_rad = 0.0;
float yaw_rate_rads = 0.0;
unsigned long last_heading_us = 0;
long still_left_ticks = 0;
long still_right_ticks = 0;
unsigned long moved_ms = 0;

void updateHeading() {
  unsigned long now_us = micros();
  if (now_us - last_heading_us < HEADING_PERIOD_US) {
    return;
  }
  float dt = (now_us - last_heading_us) * 1e-6f;
  last_heading_us = now_us;
  if (dt > 0.2f) {
    dt = 0.2f;  // first call, or loop() was blocked
  }

  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  long l = left_ticks;
  long r = right_ticks;
  if (l != still_left_ticks || r != still_right_ticks) {
    still_left_ticks = l;
    still_right_ticks = r;
    moved_ms = millis();
  }
  bool stationary = millis() - moved_ms > STILL_MS;

  float rate_dps = gyroData.gyroZ - gyro_bias[2];
  if (stationary && fabs(rate_dps) < STILL_MAX_RATE_DPS) {
    gyro_bias[2] += BIAS_TRACK_ALPHA * rate_dps;
    yaw_rate_rads = 0.0f;
  } else {
    yaw_rate_rads = rate_dps * DEG_TO_RAD;
    heading_rad += yaw_rate_rads * dt;
    heading_rad = atan2f(sinf(heading_rad), cosf(heading_rad));
  }
}

// ---------------------------------------------------------------------
// Scan timestamps. The firmware used to send zero stamps and time_node
// stamped scans on arrival - but a scan's points are on average ~100+ ms
// old by then (sweep + Wi-Fi), so while turning SLAM placed every scan at
// a later, wrong heading. micro-ROS syncs this board's clock with the
// agent's (same machine as ROS), and each scan is stamped with when its
// points were actually measured. time_node keeps a valid stamp.
// ---------------------------------------------------------------------
const unsigned long TIME_SYNC_PERIOD_MS = 10000;
unsigned long last_time_sync_ms = 0;

void syncTime() {
  rmw_uros_sync_session(100);
  last_time_sync_ms = millis();
}

// ROS time at the moment millis() read `at_ms`; zero (= "restamp me") if the
// clock is not synchronized yet.
void stampAt(builtin_interfaces__msg__Time &stamp, unsigned long at_ms) {
  if (!rmw_uros_epoch_synchronized()) {
    stamp.sec = 0;
    stamp.nanosec = 0;
    return;
  }
  int64_t ns = rmw_uros_epoch_nanos() - (int64_t)(millis() - at_ms) * 1000000LL;
  stamp.sec = (int32_t)(ns / 1000000000LL);
  stamp.nanosec = (uint32_t)(ns % 1000000000LL);
}

//Encoder
#define LEFT_ENC_A 4
#define LEFT_ENC_B 13
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33

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

// On-board emergency stop (see updateSpeedControl): no forward motion when
// a fresh reading within +-ESTOP_HALF_ANGLE_DEG of straight ahead is closer
// than ESTOP_DISTANCE_M (from the LiDAR centre; the front edge is ~0.11 m
// ahead of it, so 0.20 m leaves ~9 cm).
const float ESTOP_DISTANCE_M = 0.20;
const int ESTOP_HALF_ANGLE_DEG = 35;
const uint16_t ESTOP_MIN_VALID_MM = 30;  // closer = noise / own parts

float frontClearanceM() {
  unsigned long now = millis();
  uint16_t nearest = 0xFFFF;
  for (int a = -ESTOP_HALF_ANGLE_DEG; a <= ESTOP_HALF_ANGLE_DEG; a++) {
    int i = (a + 360) % 360;
    uint16_t d = lidarDistances[i];
    if (d < ESTOP_MIN_VALID_MM || now - lidarStamps[i] > LIDAR_MAX_AGE_MS) {
      continue;
    }
    if (d < nearest) {
      nearest = d;
    }
  }
  return nearest / 1000.0;
}

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

// Interrupts taken per encoder since boot (sent in the encoder message).
// A noisy A line fires thousands of times a second with ticks cancelling
// (+1/-1), starving loop() of CPU while the count barely moves.
volatile uint32_t left_isr_count = 0;
volatile uint32_t right_isr_count = 0;

// Read the pin straight from the GPIO input register: digitalRead() is not
// in IRAM on this core, so every interrupt also paid a flash-cache fetch.
static inline IRAM_ATTR bool readPin(uint8_t pin) {
  return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1
                  : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
}

void IRAM_ATTR leftEncoderISR() {
  left_isr_count++;
  bool a = readPin(LEFT_ENC_A);
  bool b = readPin(LEFT_ENC_B);
  left_ticks += (a == b) ? 1 : -1;
}

void IRAM_ATTR rightEncoderISR() {
  right_isr_count++;
  bool a = readPin(RIGHT_ENC_A);
  bool b = readPin(RIGHT_ENC_B);
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

  // On-board emergency stop: refuse forward motion when the LiDAR (read
  // right here, no Wi-Fi involved) sees something close ahead. The laptop's
  // obstacle check only works on scans that arrive - over a bad link they
  // came seconds late or not at all and the robots drove into things.
  // Turning stays allowed so the robot can still turn away.
  float forward = (target_left_speed + target_right_speed) / 2.0;
  if (forward > 0.0) {
    float clearance = frontClearanceM();
    if (clearance < ESTOP_DISTANCE_M) {
      target_left_speed -= forward;
      target_right_speed -= forward;
      static unsigned long last_estop_log = 0;
      if (now - last_estop_log > 1000) {
        last_estop_log = now;
        Serial.printf("[estop] obstacle %.2f m ahead - forward motion blocked\n", clearance);
      }
    }
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
  // No return: the library reports these as max range (12 m) with
  // is_obstacle = false. Stored as 12000 they passed the "> 12000" check
  // and were published as real hits 12 m away (ghost points around the map).
  lidarDistances[angleDeg] = point.is_obstacle ? (uint16_t)(point.distance * 1000.0f) : 0;
  lidarStamps[angleDeg] = millis();
}

bool readLidarByte(uint8_t &b, void *ref);  // defined with drainLidar()

// Discards the LiDAR library's log output (see setupLidar).
class NullPrint : public Print {
 public:
  size_t write(uint8_t) override { return 1; }
  size_t write(const uint8_t *, size_t size) override { return size; }
};
NullPrint lidarLogSink;

void setupLidar() {
  // The LD19 streams ~23 KB/s; the default 256-byte buffer overflows (and
  // loses points) whenever the loop blocks on Wi-Fi for more than ~10 ms.
  LidarSerial.setRxBufferSize(4096);
  LidarSerial.begin(230400, SERIAL_8N1, 16, 17);
  lidar.setResultCallback(onLidarPoint);
  lidar.setReadByteCallback(readLidarByte);  // parse from lidarBuf (see drainLidar)
  lidar.setAngleUnit(LidarAngleUnit::DEG);
  lidar.setDistanceUnit(LidarDistanceUnit::M);
  // LidarLogLevel::OFF does NOT silence the library: its logger prints when
  // message level <= current level, and OFF (5) is the highest, so OFF
  // enabled every DEBUG/TRACE line - several per packet and per skipped
  // byte. Serial (115200 baud) could not keep up, every print blocked, and
  // the whole loop ran at ~1 Hz. ERROR is the quietest real level, and the
  // output goes to a sink that discards it.
  lidar.setLogLevel(LidarLogLevel::ERROR);
  lidar.setLogOutput(lidarLogSink);
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
//
// The parser must NOT read the UART itself: readData(Stream&) pulls one byte
// per Serial.read() call, each with driver locking, and draining ~3 KB that
// way took ~350 ms per loop (loop at ~1 Hz, all sensors at ~1 Hz). Instead,
// copy everything waiting in ONE bulk read and let the parser read from RAM.
const size_t LIDAR_PACKET_BYTES = 47;
uint8_t lidarBuf[2048];
size_t lidarLen = 0;  // bytes held in lidarBuf
size_t lidarPos = 0;  // next byte the parser reads

bool readLidarByte(uint8_t &b, void *ref) {
  if (lidarPos >= lidarLen) {
    return false;
  }
  b = lidarBuf[lidarPos++];
  return true;
}

void drainLidar() {
  // Keep the unparsed tail (the start of the next packet) at the front.
  memmove(lidarBuf, lidarBuf + lidarPos, lidarLen - lidarPos);
  lidarLen -= lidarPos;
  lidarPos = 0;

  size_t n = LidarSerial.available();
  if (n > sizeof(lidarBuf) - lidarLen) {
    n = sizeof(lidarBuf) - lidarLen;
  }
  if (n > 0) {
    lidarLen += LidarSerial.read(lidarBuf + lidarLen, n);
  }

  // Each readData() consumes at least one byte, so this always ends.
  while (lidarLen - lidarPos >= LIDAR_PACKET_BYTES) {
    lidar.readData();
  }
}

void publishScan(unsigned long current_time) {
  drainLidar();  // freshest points into this scan
  unsigned long now = millis();
  // Mean measurement time of the points that go into this scan.
  unsigned long age_sum = 0;
  unsigned long age_count = 0;
  for (int i = 0; i < 360; i++) {
    // 0 = no reading; the STL also reports failed returns as ~65 m.
    bool stale = now - lidarStamps[i] > LIDAR_MAX_AGE_MS;
    bool valid = !(stale || lidarDistances[i] == 0 || lidarDistances[i] > 12000);
    scan_msg.ranges.data[i] = valid ? lidarDistances[i] / 1000.0 : INFINITY;
    if (valid) {
      age_sum += now - lidarStamps[i];
      age_count++;
    }
  }
  unsigned long mean_age_ms = age_count ? age_sum / age_count : 0;
  stampAt(scan_msg.header.stamp, now - mean_age_ms);
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
// How often a connected board pings the agent to detect a lost link. With a
// 1000 ms ping every loop() ran ~1 s (executor spin absorbing the rest of
// the second) - testing whether the ping is what paces it.
const unsigned long CONNECTED_PING_PERIOD_MS = 5000;

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
  // The scan publish keeps micro-ROS's default ACK wait. A 20 ms cap was
  // tried while Wi-Fi power save delayed ACKs; with power save off (ACKs in
  // ~8 ms) the cap left scans unacknowledged, the retry interval backed off
  // to 1024 ms, and the executor spin stalled ~1 s per loop.
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
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 0, WIFI_BSSID);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
    }
    // Own UDP transport instead of set_microros_wifi_transports() - see
    // uros_udp_read().
    rmw_uros_set_custom_transport(false, NULL,
        uros_udp_open, uros_udp_close, uros_udp_write, uros_udp_read);
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
    encoder_msg.data.data = (int32_t *)malloc(ENCODER_MSG_LEN * sizeof(int32_t));
    encoder_msg.data.size = ENCODER_MSG_LEN;
    encoder_msg.data.capacity = ENCODER_MSG_LEN;

    setupEncoders();
    setupMotors();
    setupLidar();
    Serial.println("Hardware ready. Waiting for micro-ROS agent...");
}

void publishSensors(unsigned long current_time)
{
    if (current_time - last_publish_time >= PUBLISH_PERIOD_MS) {
        last_publish_time = current_time;

        // accelData/gyroData/heading come from updateHeading() in loop().
        imu_msg.linear_acceleration.x = accelData.accelX * G_TO_MS2;
        imu_msg.linear_acceleration.y = accelData.accelY * G_TO_MS2;
        imu_msg.linear_acceleration.z = accelData.accelZ * G_TO_MS2;

        imu_msg.angular_velocity.x = (gyroData.gyroX - gyro_bias[0]) * DEG_TO_RAD;
        imu_msg.angular_velocity.y = (gyroData.gyroY - gyro_bias[1]) * DEG_TO_RAD;
        imu_msg.angular_velocity.z = yaw_rate_rads;

        // Yaw-only orientation from the on-board integrated heading (the EKF
        // fuses its change between messages - imu0_differential).
        imu_msg.orientation.x = 0.0;
        imu_msg.orientation.y = 0.0;
        imu_msg.orientation.z = sinf(heading_rad / 2.0f);
        imu_msg.orientation.w = cosf(heading_rad / 2.0f);
        imu_msg.orientation_covariance[0] = 1e3;  // roll/pitch not measured
        imu_msg.orientation_covariance[4] = 1e3;
        imu_msg.orientation_covariance[8] = 0.0005;

        // Restamped on arrival by time_node (the EKF drops measurements
        // older than ones it has already used).
        imu_msg.header.stamp.sec = 0;
        imu_msg.header.stamp.nanosec = 0;

        imu_msg.angular_velocity_covariance[0] = 0.0004;
        imu_msg.angular_velocity_covariance[4] = 0.0004;
        imu_msg.angular_velocity_covariance[8] = 0.0004;
        imu_msg.linear_acceleration_covariance[0] = 0.04;
        imu_msg.linear_acceleration_covariance[4] = 0.04;
        imu_msg.linear_acceleration_covariance[8] = 0.04;

        rcl_publish(&imu_publisher, &imu_msg, NULL);

        encoder_msg.data.data[0] = left_ticks;
        encoder_msg.data.data[1] = right_ticks;
        for (int i = 0; i < T_COUNT; i++) {
          encoder_msg.data.data[2 + i] = t_max_ms[i];
          t_max_ms[i] = 0;
        }
        encoder_msg.data.data[2 + T_COUNT] = left_isr_count;
        encoder_msg.data.data[3 + T_COUNT] = right_isr_count;
        encoder_msg.data.data[4 + T_COUNT] = read_req_max_ms;
        encoder_msg.data.data[5 + T_COUNT] = read_took_max_ms;
        encoder_msg.data.data[6 + T_COUNT] = read_inf_count;
        // micro-ROS's clock next to Arduino's: if uxr_millis() only moves in
        // whole seconds, every "10 ms" session wait lasts until the next
        // second boundary (spin ~970 ms, loop locked to 1 s).
        encoder_msg.data.data[7 + T_COUNT] = (int32_t)uxr_millis();
        encoder_msg.data.data[8 + T_COUNT] = (int32_t)millis();
        read_req_max_ms = 0;
        read_took_max_ms = 0;
        rcl_publish(&encoder_publisher, &encoder_msg, NULL);
    }

    if (current_time - last_scan_publish_time >= SCAN_PUBLISH_PERIOD_MS) {
        last_scan_publish_time = current_time;
        TIME_MAX(T_SCAN, publishScan(current_time));
    }
}

void loop()
{
    uint32_t loop_t0 = micros();
    TIME_MAX(T_DRAIN, drainLidar());
    updateHeading();  // in every state, so heading survives agent/Wi-Fi outages
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
          syncTime();
          last_agent_check = now;
          agent_state = AGENT_CONNECTED;
        } else {
          Serial.println("[ros] setup failed - retrying.");
          destroyEntities();
          agent_state = WAITING_AGENT;
        }
        break;

      case AGENT_CONNECTED:
        if (now - last_agent_check > CONNECTED_PING_PERIOD_MS) {
          last_agent_check = now;
          rmw_ret_t ping_rc = RMW_RET_OK;
          TIME_MAX(T_PING, ping_rc = rmw_uros_ping_agent(100, 3));
          if (ping_rc != RMW_RET_OK) {
            agent_state = AGENT_DISCONNECTED;
            break;
          }
        }
        // Keep the clock in step with the agent (drift, missed first sync).
        if (now - last_time_sync_ms > TIME_SYNC_PERIOD_MS) {
          syncTime();
        }
        TIME_MAX(T_SPIN, rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

        // Watchdog: stop if no cmd_vel for CMD_VEL_TIMEOUT_MS.
        if (now - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
          stopMotors();
        }
#if ENABLE_SPEED_PID
        TIME_MAX(T_PID, updateSpeedControl());
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

    uint32_t loop_ms = (micros() - loop_t0) / 1000;
    if (loop_ms > t_max_ms[T_LOOP]) t_max_ms[T_LOOP] = loop_ms;
}
