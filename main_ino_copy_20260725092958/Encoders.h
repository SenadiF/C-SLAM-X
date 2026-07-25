#ifndef ENCODERS_H
#define ENCODERS_H
#include <Arduino.h>

#define LEFT_ENC_A 4
#define LEFT_ENC_B 13
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33

extern volatile long left_ticks;
extern volatile long right_ticks;

void setupEncoders();

#endif