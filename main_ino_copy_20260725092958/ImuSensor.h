#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H
#include <FastIMU.h>

#define IMU_ADDRESS 0x69
#define G_TO_MS2 9.80665f
#define DEG_TO_RAD 0.01745329252f

extern BMI160 IMU;

void setupImu();
void publishImuData(unsigned long current_time);

#endif