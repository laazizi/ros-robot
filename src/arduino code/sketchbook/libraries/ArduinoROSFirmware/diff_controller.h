int Kp = 4;
int Ki = 100;
int Kd = 1;

typedef struct {
  double SetpointTicks;
  double Input;  
  double PreviousInput;  
  double Output;
  double Encoder; 
  double PreviousEncoder;
  double f;
}
PidInfo;

PidInfo leftPID, rightPID;

PID myPIDL(&leftPID.Input, &leftPID.Output, &leftPID.SetpointTicks,Kp,Ki,Kd, DIRECT);
PID myPIDR(&rightPID.Input, &rightPID.Output, &rightPID.SetpointTicks,Kp,Ki,Kd, DIRECT);

unsigned char moving = 0; // is the base in motion?

void resetPID(){
   leftPID.Input = 0;
   leftPID.PreviousInput = 0;
   leftPID.Output = 0;
   leftPID.SetpointTicks = 0;
   leftPID.Encoder = readEncoder(0);
   leftPID.PreviousEncoder = leftPID.Encoder;
   leftPID.f = 1;
   
   rightPID.Input = 0;
   rightPID.PreviousInput = 0;
   rightPID.Output = 0;
   rightPID.SetpointTicks = 0;
   rightPID.Encoder = readEncoder(1);
   rightPID.PreviousEncoder = rightPID.Encoder;
   rightPID.f = 1;
}


void updatePID() {
    leftPID.Encoder = readEncoder(0);
    rightPID.Encoder = readEncoder(1);
  
  if (!moving){
    if (leftPID.PreviousInput != 0 || rightPID.PreviousInput != 0){
      resetPID();
    }
    return;
  } 
    leftPID.Input = leftPID.Encoder - leftPID.PreviousEncoder;
    myPIDL.Compute();
    leftPID.PreviousEncoder = leftPID.Encoder;
    leftPID.PreviousInput = leftPID.Input;
    
    rightPID.Input = rightPID.Encoder - rightPID.PreviousEncoder;
    myPIDR.Compute();
    rightPID.PreviousEncoder = rightPID.Encoder;
    rightPID.PreviousInput = rightPID.Input;
   
  /* Set the motor speeds accordingly */
  
  setMotorSpeeds(leftPID.Output * leftPID.f, rightPID.Output * rightPID.f);
}
