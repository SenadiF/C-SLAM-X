#define LEFT_ENC_A  4
#define LEFT_ENC_B  13
#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33

void setup() {
  Serial.begin(115200);
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
}

void loop() {
  // Read raw pin voltage states
  Serial.print("Left A: "); Serial.print(digitalRead(LEFT_ENC_A));
  Serial.print(" | Left B: "); Serial.print(digitalRead(LEFT_ENC_B));
  Serial.print(" || Right A: "); Serial.print(digitalRead(RIGHT_ENC_A));
  Serial.print(" | Right B: "); Serial.println(digitalRead(RIGHT_ENC_B));
  delay(150);
}
