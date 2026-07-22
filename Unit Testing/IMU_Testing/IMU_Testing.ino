#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;


void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println("BMI160 Test");


  if (bmi160.softReset() != BMI160_OK)
  {
    Serial.println("Reset failed");
    while(1);
  }


  if (bmi160.I2cInit() != BMI160_OK)
  {
    Serial.println("BMI160 not found!");
    while(1);
  }


  Serial.println("BMI160 Connected!");
}


void loop()
{

}