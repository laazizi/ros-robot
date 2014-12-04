#include <Arduino.h>
class ArpiMotors
{
  public:  
    ArpiMotors(); // Default pin selection.
    ArpiMotors(unsigned char M2DIR, unsigned char M2PWM, unsigned char M1DIR, unsigned char M1PWM); // User-defined pin selection. 
    
    void init();
    void setM1Speed(int speed);
    void setM2Speed(int speed);
    void setSpeeds(int m2Speed, int m1Speed);
    
  private:
    unsigned char _M1DIR;
    unsigned char _M2DIR;
    unsigned char _M1PWM;
    unsigned char _M2PWM;

};

