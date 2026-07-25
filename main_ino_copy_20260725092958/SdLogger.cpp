#include "SdLogger.h"
#include <SPI.h>
#include <SD.h>
#include "Encoders.h"
#include "LidarReader.h"
#include "RosComms.h"

#define LOG_FILENAME "/buffer.bin"

bool sdReady = false;


struct LogRecord {
  unsigned long timestamp;
  long left_ticks;
  long right_ticks;
  uint16_t lidar[360];
};

bool setupSd() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card init FAILED — buffering disabled for this session.");
    sdReady = false;
    return false;
  }
  Serial.println("SD card initialized.");
  sdReady = true;
  return true;
}

void logSensorDataToSD() {
  if (!sdReady) return;

  File f = SD.open(LOG_FILENAME, FILE_APPEND);
  if (!f) {
    Serial.println("Failed to open log file for writing.");
    return;
  }

  LogRecord record;
  record.timestamp = millis();
  record.left_ticks = left_ticks;
  record.right_ticks = right_ticks;
  memcpy(record.lidar, lidarDistances, sizeof(record.lidar));

  f.write((uint8_t*)&record, sizeof(LogRecord));  
  f.close();
}

void uploadBufferedLogs() {
  if (!sdReady) return;

  File f = SD.open(LOG_FILENAME, FILE_READ);
  if (!f) {
    Serial.println("No buffered log to upload.");
    return;
  }

  Serial.println("Uploading buffered logs...");
  int recordsUploaded = 0;
  LogRecord record;

  while (f.available() >= (int)sizeof(LogRecord)) {
    f.read((uint8_t*)&record, sizeof(LogRecord));

    // Republish ticks 
    encoder_msg.data.data[0] = record.left_ticks;
    encoder_msg.data.data[1] = record.right_ticks;
    rcl_publish(&encoder_publisher, &encoder_msg, NULL);

    
    memcpy(lidarDistances, record.lidar, sizeof(record.lidar));
    publishLidarScan(record.timestamp); \

    recordsUploaded++;
    delay(5);
  }

  f.close();
  SD.remove(LOG_FILENAME);

  Serial.print("Buffer uploaded and cleared. Records replayed: ");
  Serial.println(recordsUploaded);
}