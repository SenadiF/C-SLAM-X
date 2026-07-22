#include <DFRobot_BMI160.h>

DFRobot_BMI160 bmi160;


void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println("BMI160 Test");


  
  if (bmi160.I2cInit(0x69) != BMI160_OK)
  {
    Serial.println("BMI160 init failed");
    while(1);
  }


  Serial.println("BMI160 Connected!");
}


void loop()
{

}