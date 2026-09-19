#include <Wire.h>
#include "DFRobot_BMI160.h"

DFRobot_BMI160 bmi160;
const int8_t i2c_addr = 0x69; // Change to 0x68 if your SDO pin is pulled low

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize the I2C bus
  Wire.begin();

  // Reset and verify connection
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("BMI160 reset failed! Check wiring.");
    while (1);
  }

  // Initialize communication with the specified address
  if (bmi160.I2cInit(i2c_addr) != BMI160_OK) {
    Serial.println("BMI160 initialization failed! Address incorrect?");
    while (1);
  }

  Serial.println("BMI160 connected and verified successfully!");
}

void loop() {
  int16_t accelGyro[6] = {0};
  
  // Read accelerometer and gyroscope data
  if (bmi160.getAccelGyroData(accelGyro) == 0) {
    Serial.print("Gyro (rad/s): ");
    for (int i = 0; i < 3; i++) {
      Serial.print(accelGyro[i] * 3.14 / 180.0, 2);
      Serial.print("\t");
    }
    
    Serial.print("Accel (g): ");
    for (int i = 3; i < 6; i++) {
      Serial.print(accelGyro[i] / 16384.0, 2);
      Serial.print("\t");
    }
    Serial.println();
  } else {
    Serial.println("Failed to read sensor data.");
  }
  
  delay(500);
}
