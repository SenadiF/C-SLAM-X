#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

enum RobotState {
  INITIALIZE,
  NORMAL_MODE,
  LOCAL_EXPLORATION,
  SYNC_MODE
};

extern RobotState currentState;

void setState(RobotState newState);
void updateStateMachine();

#endif