#include "ArpiMotors.h"

ArpiMotors::ArpiMotors()
{
  _M1DIR = 4; // Motor 1 direction pin
  _M2DIR = 7; // Motor 2 direction pin
  _M1PWM = 5; // Motor 1 PWM pin
  _M2PWM = 6; // Motor 2 PWM pin
}

ArpiMotors::ArpiMotors(unsigned char M2DIR, unsigned char M2PWM, unsigned char M1DIR, unsigned char M1PWM)
{
  _M1DIR = M1DIR;
  _M2DIR = M2DIR;
  _M1PWM = M1PWM; 
  _M2PWM = M2PWM;
}


// Initialize motor pins
void ArpiMotors::init()
{
  pinMode(_M1DIR,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M2DIR,OUTPUT);
  pinMode(_M2PWM,OUTPUT);

}

// Set motor 1 speed in PWM value [-255,255]
void ArpiMotors::setM1Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;

  analogWrite(_M1PWM,speed);

  if (reverse)
    digitalWrite(_M1DIR,HIGH);
  else
    digitalWrite(_M1DIR,LOW);
}

// Set motor 2 speed in PWM value [-255,255]
void ArpiMotors::setM2Speed(int speed)
{
  unsigned char reverse = 0;

  if (speed < 0)
  {
    speed = -speed;  // Make speed a positive quantity
    reverse = 1;  // Preserve the direction
  }
  if (speed > 255)  // Max PWM dutycycle
    speed = 255;

  analogWrite(_M2PWM,speed);  

  if (reverse)
    digitalWrite(_M2DIR,HIGH);
  else
    digitalWrite(_M2DIR,LOW);
}

// Set both motor speeds in PWM value [-255,255]
void ArpiMotors::setSpeeds(int m2Speed, int m1Speed)
{
  setM1Speed(m1Speed);
  setM2Speed(m2Speed);
}
