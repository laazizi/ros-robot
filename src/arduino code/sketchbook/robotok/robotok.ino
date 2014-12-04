#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
  #include <PWM.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
 

#define pwmM1 2
#define pwmM2 3
#define dirM1 A2
#define dirM2 A1

int32_t frequency = 200;
ros::NodeHandle  nh;

Servo servo;

void servo_cL( const std_msgs::UInt16& cmd_msg){
  
  if (cmd_msg.data < 512)
  {
    // reverse rotation
    int V = -(cmd_msg.data - 511) / 2;
    digitalWrite(dirM1,HIGH); 
    analogWrite(pwmM1,V); 
    
  }
  else
  {
    // forward rotation
    int V = (cmd_msg.data - 512) / 2;
    digitalWrite(dirM1,LOW); 
    analogWrite(pwmM1,V); 
  }
  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void servo_cR( const std_msgs::UInt16& cmd_msg){
 
    if (cmd_msg.data < 512)
  {
    // reverse rotation
    int V = -(cmd_msg.data - 511) / 2;
    digitalWrite(dirM2,HIGH);  
    analogWrite(pwmM2, V);
  }
  else
  {
    // forward rotation
    int V = (cmd_msg.data - 512) / 2;
    digitalWrite(dirM2,LOW); 
    analogWrite(pwmM2, V);
     
  }
  
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> subL("servoL", servo_cL);
ros::Subscriber<std_msgs::UInt16> subR("servoR", servo_cR);

void setup(){
   InitTimersSafe(); 
 TCCR3B = TCCR3B & B11111000 | B00000011;    // set timer 3 divisor to    64 for PWM frequency of   490.20 Hz
  pinMode(13, OUTPUT);
   
//setPwmFrequency(3,200);
//setPwmFrequency(11,200);
  ////sets the frequency for the specified pin
   //bool a = SetPinFrequencySafe(3, frequency);
      
 //  bool b = SetPinFrequencySafe(11, frequency);
  //  bool c = SetPinFrequencySafe(6, frequency);
   //  bool d = SetPinFrequencySafe(7, frequency);
 


  nh.initNode();
  nh.subscribe(subL);
  nh.subscribe(subR);
   
  
}

void loop(){
  nh.spinOnce();
    delay(1);
}
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

