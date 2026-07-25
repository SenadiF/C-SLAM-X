#include "ImuSensor.h"
#include "RosComms.h"

BMI160 IMU;
calData calib = {0};
AccelData accelData;
GyroData gyroData;

void setupImu() {
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("IMU initialization failed: ");
    Serial.println(err);
    while (true);
  }
  Serial.println("BMI160 Initialized.");
}

void publishImuData(unsigned long current_time) {
  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  imu_msg.linear_acceleration.x = accelData.accelX * G_TO_MS2;
  imu_msg.linear_acceleration.y = accelData.accelY * G_TO_MS2;
  imu_msg.linear_acceleration.z = accelData.accelZ * G_TO_MS2;

  imu_msg.angular_velocity.x = gyroData.gyroX * DEG_TO_RAD;
  imu_msg.angular_velocity.y = gyroData.gyroY * DEG_TO_RAD;
  imu_msg.angular_velocity.z = gyroData.gyroZ * DEG_TO_RAD;

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
}