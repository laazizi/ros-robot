/* *************************************************************
   Encoder definitions
   
   Add a "#if defined" block to this file to include support for
   a particular encoder board or library. Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.
   
   ************************************************************ */
   
#ifdef USE_BASE

#if defined ROBOGAIA
  /* The Robogaia Mega Encoder shield */
  #include "encode.h"
#include <Encoder.h>
 Encoder knobLeft(2, 7);
Encoder knobRight(3, 8);
  /* Create the encoder shield object */
  encode encoders = encode(4); // Initializes the Mega Encoder Counter in the 4X Count mode
  
  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    
     long newLeft, newRight;
      newLeft = knobLeft.read();
      newRight = knobRight.read();
         if (i == LEFT) return newLeft;
    else return newRight;
 ;
    
    //else return encoders.XAxisGetCount();
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    if (i == LEFT) return encoders.YAxisReset();
    else return encoders.XAxisReset();
  }
#else
  #error A encoder driver must be selected!
#endif

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}

#endif

