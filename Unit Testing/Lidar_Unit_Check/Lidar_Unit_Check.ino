#include "LidarReader.h"

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting LiDAR test...");
  setupLidar();
}

void loop() {
  updateLidar();

  
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    Serial.println("---- LiDAR snapshot ----");
    Serial.print("0 deg: ");   Serial.println(lidarDistances[0]);
    Serial.print("90 deg: ");  Serial.println(lidarDistances[90]);
    Serial.print("180 deg: "); Serial.println(lidarDistances[180]);
    Serial.print("270 deg: "); Serial.println(lidarDistances[270]);
  }
}