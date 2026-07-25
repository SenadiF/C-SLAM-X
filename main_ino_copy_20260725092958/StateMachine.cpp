#include "StateMachine.h"
#include <WiFi.h>
#include "MotorControl.h"
#include "GapFollow.h"
#include "SdLogger.h"
#include "LidarReader.h"

RobotState currentState = INITIALIZE;
unsigned long stateEnteredTime = 0;

void setState(RobotState newState) {
  if (newState == currentState) return;
  currentState = newState;
  stateEnteredTime = millis();
  Serial.print("State changed to: ");
  Serial.println(newState); 
}

void updateStateMachine() {
  updateLidar(); 

  bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  bool cmdVelFresh = (millis() - last_cmd_vel_time < CMD_VEL_TIMEOUT_MS);

  switch (currentState) {
    case INITIALIZE:
      if (wifiConnected) setState(NORMAL_MODE);
      break;

    case NORMAL_MODE:
      if (!wifiConnected || !cmdVelFresh) {
        stopMotors();
        setState(LOCAL_EXPLORATION);
      } else {
        updateMotorPID();
      }
      break;

    case LOCAL_EXPLORATION:
      if (wifiConnected) {
        setState(SYNC_MODE);
      } else {
        followTheGap();
        updateMotorPID();       
        logSensorDataToSD();
      }
      break;

    case SYNC_MODE:
      uploadBufferedLogs();
      setState(NORMAL_MODE);
      break;
  }
}