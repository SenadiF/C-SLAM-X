#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H
#include <Arduino.h>

#define LEFT_MOTOR_IN1 25
#define LEFT_MOTOR_IN2 26
#define RIGHT_MOTOR_IN1 27
#define RIGHT_MOTOR_IN2 14

#define LEFT_MOTOR_REVERSED  false
#define RIGHT_MOTOR_REVERSED false

#define PWM_FREQ 5000
#define PWM_RESOLUTION 8

extern unsigned long last_cmd_vel_time;
extern const unsigned long CMD_VEL_TIMEOUT_MS;

void setupMotors();
void stopMotors();
void driveMotor(int pinForward, int pinBackward, float speed, bool reversed);
void cmd_vel_callback(const void *msgin);
void updateMotorPID();

#endif