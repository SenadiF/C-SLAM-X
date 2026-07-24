

// ---- Motor driver pins ----
#define LEFT_MOTOR_IN1  25
#define LEFT_MOTOR_IN2  26
#define RIGHT_MOTOR_IN1 27
#define RIGHT_MOTOR_IN2 14

// ---- Encoder pins ----
#define LEFT_ENC_A  4
#define LEFT_ENC_B  13
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33

volatile long left_ticks = 0;
volatile long right_ticks = 0;

void IRAM_ATTR leftEncoderISR() {
  bool a = digitalRead(LEFT_ENC_A);
  bool b = digitalRead(LEFT_ENC_B);
  left_ticks += (a == b) ? 1 : -1;
}

void IRAM_ATTR rightEncoderISR() {
  bool a = digitalRead(RIGHT_ENC_A);
  bool b = digitalRead(RIGHT_ENC_B);
  right_ticks += (a == b) ? 1 : -1;
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void driveForward() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, HIGH);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  stopMotors();

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);

  Serial.println("Setup done. Lift wheels off the ground before testing!");
  delay(2000);
}

void loop() {
  Serial.println("Driving forward for 2 seconds...");
  driveForward();
  delay(2000);
  stopMotors();

  Serial.print("Left ticks: ");
  Serial.print(left_ticks);
  Serial.print("   Right ticks: ");
  Serial.println(right_ticks);

  delay(3000); 
}