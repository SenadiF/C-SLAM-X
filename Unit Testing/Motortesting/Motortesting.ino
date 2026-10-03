#define LEFT_IN1   2
#define LEFT_IN2   15
#define RIGHT_IN3  14
#define RIGHT_IN4  27

void forward()
{
  // Left motor forward
  digitalWrite(LEFT_IN1,HIGH);
  digitalWrite(LEFT_IN2,LOW);

  // Right motor forward
  digitalWrite(RIGHT_IN3,LOW);
  digitalWrite(RIGHT_IN4, HIGH);
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