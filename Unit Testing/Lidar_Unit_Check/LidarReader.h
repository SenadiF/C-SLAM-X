#ifndef LIDAR_READER_H
#define LIDAR_READER_H
#include <Arduino.h>


#define LIDAR_RX_PIN 16   // ESP32 RX - LiDAR TX
#define LIDAR_TX_PIN 17   // ESP32 TX - LiDAR RX
#define SerialLidar Serial2


extern uint16_t lidarDistances[360];

void setupLidar();
void updateLidar();  
void publishLidarScan(unsigned long current_time); 

#endif