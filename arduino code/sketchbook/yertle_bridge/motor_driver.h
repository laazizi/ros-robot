/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/
#include "Arduino.h"
#include "commands.h"

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#define P_RFWD 8
#define P_RREV 11
#define P_RENA 9

#define P_LFWD 12
#define P_LREV 13
#define P_LENA 10

#define EN_BAR 0
