#include "encoder_driver.h"
long lcoder = 0;
long rcoder = 0;
long lprev = 0;
long rprev = 0;


//////////////////////////////////////////////////////////////////////
void doLEncoder(){
//////////////////////////////////////////////////////////////////////
	// note that you could get 4x the resolution by catching rising and fallling edges of A and B.
	if (digitalRead(LWHEEL_B)) {
		lcoder += 1;
	} else {
		lcoder -= 1;
	}
}

//////////////////////////////////////////////////////////////////////
void doREncoder(){
//////////////////////////////////////////////////////////////////////
	if (digitalRead(RWHEEL_B)){
		rcoder += 1;
	} else {
		rcoder -= 1;
	}
}


long readEncoder(int i){
	if (i==LEFT) return lcoder;
	else return rcoder;
}
void resetEncoder(int i){
	if (i==LEFT) lcoder = 0;
	else rcoder = 0;

}
void resetEncoders(){
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
}

void initEncoders() {
	  pinMode(LWHEEL_B, INPUT);
	  pinMode(RWHEEL_B, INPUT);
	  attachInterrupt(LWHEEL_A_INT, doLEncoder, RISING);   //init the interrupt mode
	  attachInterrupt(RWHEEL_A_INT, doREncoder, RISING);
}


