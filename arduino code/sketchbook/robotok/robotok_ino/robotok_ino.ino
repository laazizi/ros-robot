#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
 #include <PWM.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>

#define pwmM1 5
#define pwmM2 6
#define dirM1 A2
#define dirM2 A1
 LogMsgType log_msg

const byte encoder0pinA = 2;//A pin -> the interrupt pin 2
const byte encoder0pinB = 4;//B pin -> the digital pin 4
byte encoder0PinALast;
int duration;//the number of the pulses
boolean Direction;//the rotation direction 

 const byte encoder0pinA2 = 3;//A pin -> the interrupt pin 2
const byte encoder0pinB2 = 7;//B pin -> the digital pin 4
byte encoder0PinALast2;
int duration2;//the number of the pulses
boolean Direction2;//the rotation direction 
 
ros::NodeHandle  nh;
std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("encoder", &temp_msg);

std_msgs::Float32 temp_msg2;
ros::Publisher pub_temp2("encoder2", &temp_msg2);

Servo servo;
void EncoderInit()
{
  Direction = true;//default -> Forward  
  pinMode(encoder0pinB,INPUT);  
  attachInterrupt(0, wheelSpeed, CHANGE);//int.0 
  
   Direction2 = true;//default -> Forward  
  pinMode(encoder0pinB2,INPUT);  
  attachInterrupt(0, wheelSpeed2, CHANGE);//int.1 
  
}

void wheelSpeed2()
{
  int Lstate2 = digitalRead(encoder0pinA2);
  if((encoder0PinALast2 == LOW) && Lstate2==HIGH)
  {
    int val2 = digitalRead(encoder0pinB2);
    
    if(val2 == LOW && Direction2)
    {
      Direction2 = false; //Reverse
    }
    else if(val2 == HIGH && !Direction2)
    {
      Direction2 = true;  //Forward
    }
  }
  encoder0PinALast2 = Lstate2;
   
  if(!Direction2)  duration2++;
  else  duration2--;
   temp_msg2.data = duration2;
      pub_temp2.publish(&temp_msg2);
      
    }
void wheelSpeed()
{
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
  }
  encoder0PinALast = Lstate;
   
  if(!Direction)  duration++;
  else  duration--;
   temp_msg.data = duration;
      pub_temp.publish(&temp_msg);
   
    }
    
    
    
    
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
   //InitTimersSafe(); 
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
  pinMode(13, OUTPUT);
    EncoderInit();//Initialize the module
 


  nh.initNode();
  nh.advertise(pub_temp);
    nh.advertise(pub_temp2);
  nh.subscribe(subL);
  nh.subscribe(subR);
   
  
}

void loop(){
  nh.spinOnce();
   duration = 0;
     duration2 = 0;
    delay(1);
}
 

