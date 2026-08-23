#define LEFT_IN1   2
#define LEFT_IN2   15
#define RIGHT_IN3  27
#define RIGHT_IN4  14

void forward()
{
  // Left motor forward
  digitalWrite(LEFT_IN1,LOW);
  digitalWrite(LEFT_IN2,HIGH);

  // Right motor forward
  digitalWrite(RIGHT_IN3,HIGH);
  digitalWrite(RIGHT_IN4, LOW);
}

void setup()
{
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN3, OUTPUT);
  pinMode(RIGHT_IN4, OUTPUT);

  forward();
}

void loop()
{
}