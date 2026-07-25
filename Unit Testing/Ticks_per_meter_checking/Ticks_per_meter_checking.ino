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

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, CHANGE);
  Serial.println("Check the Robots distance");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'r') {
      left_ticks = 0;
      right_ticks = 0;
      Serial.println("Moving the robot one meter");
    }
  }
  Serial.print("Left: "); Serial.print(left_ticks);
  Serial.print("   Right: "); Serial.println(right_ticks);
  delay(300);
}