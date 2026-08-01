#include <Wire.h>
#include <FastIMU.h>

#define IMU_ADDRESS 0x69

BMI160 IMU;
calData calib = {0};

AccelData accelData;
GyroData gyroData;

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("Starting BMI160 Test...");

  Wire.begin(21, 22);
  Wire.setClock(100000);

  int err = IMU.init(calib, IMU_ADDRESS);

  if (err != 0) {
    Serial.print("IMU init failed! Error: ");
    Serial.println(err);

    while (1) {
      delay(1000);
    }
  }

  Serial.println("BMI160 initialized successfully!");
}

void loop() {

  IMU.update();

  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);

  Serial.println("----------------");

  Serial.print("Accel X (g): ");
  Serial.println(accelData.accelX, 4);

  Serial.print("Accel Y (g): ");
  Serial.println(accelData.accelY, 4);

  Serial.print("Accel Z (g): ");
  Serial.println(accelData.accelZ, 4);

  Serial.print("Gyro X (deg/s): ");
  Serial.println(gyroData.gyroX, 4);

  Serial.print("Gyro Y (deg/s): ");
  Serial.println(gyroData.gyroY, 4);

  Serial.print("Gyro Z (deg/s): ");
  Serial.println(gyroData.gyroZ, 4);

  delay(500);
}