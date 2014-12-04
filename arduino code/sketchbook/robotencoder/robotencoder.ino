#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
 //#include <PWM.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <Encoder.h>
#define pwmM1 5
#define pwmM2 6
#define dirM1 A2
#define dirM2 A1
 Encoder knobLeft(2, 7);
Encoder knobRight(3, 8);
 
 #include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

 
 
ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";



//std_msgs::Float32 temp_msg;
//ros::Publisher pub_temp("encoder", &temp_msg);

//std_msgs::Float32 temp_msg2;
//ros::Publisher pub_temp2("encoder2", &temp_msg2);
long positionLeft  = -999;
long positionRight = -999;

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
   //InitTimersSafe(); 
//TCCR0B = TCCR0B & B11111000 | B00000010;    // set timer 0 divisor to     8 for PWM frequency of  7812.50 Hz
  pinMode(13, OUTPUT);
    
 


  nh.initNode();
  broadcaster.init(nh);
 // nh.advertise(pub_temp);
 // nh.advertise(pub_temp2);
  nh.subscribe(subL);
  nh.subscribe(subR);
   
  
}

void loop(){
  nh.spinOnce();
  long newLeft, newRight;
  newLeft = knobLeft.read();
  newRight = knobRight.read();
//  if (newLeft != positionLeft || newRight != positionRight) {
  double dx = 0.2;
  double dtheta = 0.18;
  x = newLeft*0.001;
  y = newRight*0.001;
  theta = dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;
    
  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();
  
   broadcaster.sendTransform(t);
  //  temp_msg.data = newLeft;
    //  pub_temp.publish(&temp_msg);
      
    //temp_msg2.data = newRight;
      //pub_temp2.publish(&temp_msg2);
    positionLeft = newLeft;
    positionRight = newRight;
    delay(5);
 //}
}

