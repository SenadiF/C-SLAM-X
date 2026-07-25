#include "MotorControl.h"
#include "RosComms.h"
#include "Encoders.h"

unsigned long last_cmd_vel_time = 0;
const unsigned long CMD_VEL_TIMEOUT_MS = 500;

long prev_left_ticks = 0;
long prev_right_ticks = 0;
unsigned long last_pid_time = 0;
const unsigned long PID_PERIOD_MS = 50;

float target_left_speed = 0.0;
float target_right_speed = 0.0;

const float KP = 50.0;
const float TICKS_PER_METER = 3480.0; // dummy value — replace after calibration
const float WHEEL_BASE_M = 0.099;
const float MAX_WHEEL_SPEED_MS = 0.3;

void stopMotors() {
  ledcWrite(LEFT_MOTOR_IN1, 0);  ledcWrite(LEFT_MOTOR_IN2, 0);
  ledcWrite(RIGHT_MOTOR_IN1, 0); ledcWrite(RIGHT_MOTOR_IN2, 0);
}

void driveMotor(int pinForward, int pinBackward, float speed, bool reversed) {
  if (reversed) speed = -speed;
  int duty = (int)(fabs(speed) / MAX_WHEEL_SPEED_MS * 255.0);
  duty = constrain(duty, 0, 255);

  if (speed >= 0) {
    ledcWrite(pinForward, 0);
    ledcWrite(pinBackward, duty);
  } else {
    ledcWrite(pinForward, duty);
    ledcWrite(pinBackward, 0);
  }
}

void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  last_cmd_vel_time = millis();

  float linear = msg->linear.x;
  float angular = msg->angular.z;

  target_left_speed  = linear - (angular * WHEEL_BASE_M / 2.0);
  target_right_speed = linear + (angular * WHEEL_BASE_M / 2.0);
}

void setupMotors() {
  ledcAttach(LEFT_MOTOR_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(LEFT_MOTOR_IN2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_IN1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_MOTOR_IN2, PWM_FREQ, PWM_RESOLUTION);
  stopMotors();
}

void updateMotorPID() {
  if (millis() - last_pid_time < PID_PERIOD_MS) return;
  float dt = (millis() - last_pid_time) / 1000.0;
  last_pid_time = millis();

  if (millis() - last_cmd_vel_time > CMD_VEL_TIMEOUT_MS) {
    stopMotors();
    return;
  }

  float actual_left  = (left_ticks - prev_left_ticks) / TICKS_PER_METER / dt;
  float actual_right = (right_ticks - prev_right_ticks) / TICKS_PER_METER / dt;
  prev_left_ticks = left_ticks;
  prev_right_ticks = right_ticks;

  float left_output  = target_left_speed  + KP * (target_left_speed - actual_left) * dt;
  float right_output = target_right_speed + KP * (target_right_speed - actual_right) * dt;

  driveMotor(LEFT_MOTOR_IN1, LEFT_MOTOR_IN2, left_output, LEFT_MOTOR_REVERSED);
  driveMotor(RIGHT_MOTOR_IN1, RIGHT_MOTOR_IN2, right_output, RIGHT_MOTOR_REVERSED);
}