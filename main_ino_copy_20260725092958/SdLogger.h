#ifndef SD_LOGGER_H
#define SD_LOGGER_H
#include <Arduino.h>

#define SD_CS_PIN 5

bool setupSd();
void logSensorDataToSD();
void uploadBufferedLogs();

#endif