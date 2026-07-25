#include "GapFollow.h"
#include "LidarReader.h"
#include "MotorControl.h"

#define SAFE_DISTANCE_MM 400   // stop/turn if something is closer than this 
#define EXPLORE_SPEED_MS 0.08  

void followTheGap() {
  int bestGapStart = 0, bestGapLen = 0;
  int currentStart = -1, currentLen = 0;

  for (int i = 0; i < 360; i++) {
    bool clear = (lidarDistances[i] == 0) || (lidarDistances[i] > SAFE_DISTANCE_MM);
   
    if (clear) {
      if (currentStart == -1) currentStart = i;
      currentLen++;
    } else {
      if (currentLen > bestGapLen) { bestGapLen = currentLen; bestGapStart = currentStart; }
      currentStart = -1;
      currentLen = 0;
    }
  }
  if (currentLen > bestGapLen) { bestGapLen = currentLen; bestGapStart = currentStart; }

  if (bestGapLen == 0) {
   
    target_left_speed = 0;
    target_right_speed = 0;
    return;
  }

  int gapCenterAngle = (bestGapStart + bestGapLen / 2) % 360;

  float angleError = gapCenterAngle;
  if (angleError > 180) angleError -= 360; 

  target_left_speed  = EXPLORE_SPEED_MS - (angleError * 0.0015);
  target_right_speed = EXPLORE_SPEED_MS + (angleError * 0.0015);
}