#include "ArpiMotors.h"

ArpiMotors drive;

// Initialize motors
void initMotorController() {
  drive.init();
}

// Set one motor speed
void setMotorSpeed(int i, int spd) {
  if (i == LEFT) drive.setM2Speed(spd);
  else drive.setM1Speed(spd);
}

// Set both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}



