#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;

int16_t accelGyro[6];


void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println("BMI160 Starting");


  if (bmi160.I2cInit(0x69) != BMI160_OK)
  {
    Serial.println("BMI160 init failed");
    while(1);
  }

  Serial.println("BMI160 Connected!");
}


void loop()
{
  bmi160.getAccelGyroData(accelGyro);


  Serial.print("Accel X: ");
  Serial.print(accelGyro[0]);

  Serial.print("\tY: ");
  Serial.print(accelGyro[1]);

  Serial.print("\tZ: ");
  Serial.print(accelGyro[2]);


  Serial.print("\t | Gyro X: ");
  Serial.print(accelGyro[3]);

  Serial.print("\tY: ");
  Serial.print(accelGyro[4]);

  Serial.print("\tZ: ");
  Serial.println(accelGyro[5]);


  delay(100);
}