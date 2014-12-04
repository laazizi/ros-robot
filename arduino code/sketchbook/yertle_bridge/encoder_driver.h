/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
#include "Arduino.h"
#include "commands.h"
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
void initEncoders();

#define RWHEEL_A_INT 0
#define RWHEEL_B 4

#define LWHEEL_A_INT 1
#define LWHEEL_B 5

