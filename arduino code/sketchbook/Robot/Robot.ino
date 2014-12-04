#include <Servo.h> 
#include <TimeInfo.h>
#include <RobotParams.h>
#include <OdometricLocalizer.h>
#include <SpeedController.h>
#include <BatteryMonitor.h>
#include <Messenger.h>

#define c_UpdateIntervalForInit 100  // update interval during the initialization phase; in millisecs
#define c_UpdateIntervalNormal 20  // update interval for normal operation after the initialization phase; in millisecs

const int c_LeftServoOutPin = 8;    // Output pin that the servo is attached to
const int c_RightServoOutPin = 9;    // Output pin that the servo is attached to

Servo _LeftServo;
Servo _RightServo;

#define c_MaxMotorCV 45      // half servo range. The allowed values reach from 0 - c_MaxMotorCV to 0 + c_MaxMotorCV

// container for robot params wheel diameter [m], trackwidth [m], ticks per revolution
RobotParams _RobotParams = RobotParams();
TimeInfo _TimeInfo = TimeInfo();

// Quadrature encoders
// Left encoder
#define c_LeftEncoderPinA 33
#define c_LeftEncoderPinB 34
#define LeftEncoderIsReversed
volatile bool _LeftEncoderBSet;
volatile long _LeftEncoderTicks = 0;

// Right encoder
#define c_RightEncoderPinA 35
#define c_RightEncoderPinB 36
//#define RightEncoderIsReversed
volatile bool _RightEncoderBSet;
volatile long _RightEncoderTicks = 0;

OdometricLocalizer _OdometricLocalizer(&_RobotParams, &_TimeInfo);
SpeedController _SpeedController(&_OdometricLocalizer, &_RobotParams, &_TimeInfo);

#define c_ScaledBatteryVInPin 8 // analog input pin for the battery voltage divider
#define c_VInToVBatteryRatio 2.921
BatteryMonitor _BatteryMonitor(c_ScaledBatteryVInPin, c_VInToVBatteryRatio);

// Instantiate Messenger object with the message function and the default separator (the space character)
Messenger _Messenger = Messenger();

bool _IsInitialized = false;
int _UpdateInterval;  // update interval in milli seconds

void setup()
{
  SetupEncoders();
  _Messenger.attach(OnMssageCompleted);

  _LeftServo.attach(c_LeftServoOutPin);  // attaches the servo on specified pin to the servo object 
  _RightServo.attach(c_RightServoOutPin);  // attaches the servo on specified pin to the servo object 

  _LeftServo.write(90);
  _RightServo.write(90);

  delay(100);
  _TimeInfo.Update();
}

void SetupEncoders()
{
  // Quadrature encoders
  // Left encoder
  pinMode(c_LeftEncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_LeftEncoderPinB, INPUT_PULLUP);      // sets pin B as input
  attachInterrupt(c_LeftEncoderPinA, HandleLeftMotorInterruptA, RISING);

  // Right encoder
  pinMode(c_RightEncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_RightEncoderPinB, INPUT_PULLUP);      // sets pin B as input
  attachInterrupt(c_RightEncoderPinA, HandleRightMotorInterruptA, RISING);
  
  _UpdateInterval = c_UpdateIntervalForInit; // We use a slow update interval during the initialization phase 
}

void loop()
{
  ReadSerial();

  unsigned long milliSecsSinceLastUpdate = millis() - _TimeInfo.LastUpdateMillisecs;
  if(milliSecsSinceLastUpdate >= _UpdateInterval)
  {
    //Serial.println(milliSecsSinceLastUpdate);
    // time for another update
    _TimeInfo.Update();
    if (_IsInitialized)
    {
      _UpdateInterval = c_UpdateIntervalNormal;
      DoWork();
    }
    else
    {
      _UpdateInterval = c_UpdateIntervalForInit; // We use a slow update interval during the initialization phase 
      RequestInitialization();
      if (_IsInitialized)
      {
        // intialization is complete
        Serial.print("initialized");
        Serial.print("\n");
      }
    }
  }
}


void DoWork()
{
  _OdometricLocalizer.Update(_LeftEncoderTicks, _RightEncoderTicks);
  _BatteryMonitor.Update();
  //_SpeedController.Update(_BatteryMonitor.VoltageIsTooLow);
  _SpeedController.Update(false);
  IssueCommands();
    
  Serial.print("o"); // o indicates odometry message
  Serial.print("\t");
  Serial.print(int(_OdometricLocalizer.X * 1000));
  Serial.print("\t");
  Serial.print(int(_OdometricLocalizer.Y * 1000));
  Serial.print("\t");
  Serial.print(int(_OdometricLocalizer.Heading * 1000));
  Serial.print("\t");
  Serial.print(int(_OdometricLocalizer.V * 1000));
  Serial.print("\t");
  Serial.print(int(_OdometricLocalizer.Omega * 1000));

/*
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.VLeft);
  Serial.print("\t");
  Serial.print(_OdometricLocalizer.VRight);
*/
  Serial.print("\t");
  Serial.print(_LeftEncoderTicks);
  Serial.print("\t");
  Serial.print(_RightEncoderTicks);
  Serial.print("\n");

  Serial.print("b\t"); // b indicates battery info message
  Serial.print(_BatteryMonitor.BatteryVoltage);
  //Serial.print("\t");
  //Serial.print(_BatteryMonitor.VoltageIsTooLow);
  Serial.print("\n");
}

void RequestInitialization()
{
    _IsInitialized = true;

    if (!_RobotParams.IsInitialized)
    {
      _IsInitialized = false;

      Serial.print("InitializeDriveGeometry"); // requesting initialization of the parameters of the differential drive needed for odometry calculations
      Serial.print("\n");
      return;
    }

    if (!_SpeedController.IsInitialized)
    {
      _IsInitialized = false;

      Serial.print("InitializeSpeedController"); // requesting initialization of the speed controller
      Serial.print("\n");
      return;
    }
    
    if (!_BatteryMonitor.IsInitialized)
    {
      _IsInitialized = false;

      Serial.print("InitializeBatteryMonitor"); // requesting initialization of the battery monitor
      Serial.print("\n");
      return;
    }
}

void IssueCommands()
{
  float normalizedRightMotorCV, normalizedLeftMotorCV;
  
  normalizedLeftMotorCV = _SpeedController.NormalizedLeftCV;
  normalizedRightMotorCV = _SpeedController.NormalizedRightCV;
  
  Serial.print("Speed: ");
  Serial.print(_SpeedController.CommandedVelocity);
  Serial.print("\t");
  Serial.print(_SpeedController.CommandedAngularVelocity);
  Serial.print("\t");
  Serial.print(_SpeedController.LeftError);
  Serial.print("\t");
  Serial.print(_SpeedController.RightError);
  Serial.print("\t");
  Serial.print(_SpeedController.TurnError);
  Serial.print("\t");
  Serial.print(normalizedLeftMotorCV);
  Serial.print("\t");
  Serial.print(normalizedRightMotorCV);
  Serial.print("\n");
  /*
  */
  
  float leftServoValue = mapFloat(normalizedLeftMotorCV, -1, 1, 90.0 - c_MaxMotorCV, 90.0 + c_MaxMotorCV);     // scale it to use it with the servo (value between 0 and 180) 
  float rightServoValue = mapFloat(normalizedRightMotorCV, -1, 1, 90.0 - c_MaxMotorCV, 90.0 + c_MaxMotorCV);     // scale it to use it with the servo (value between 0 and 180) 
 
  Serial.print("Servos: ");
  Serial.print(leftServoValue);
  Serial.print("\t");
  Serial.print(rightServoValue);
  Serial.print("\n");
  /*
  */

  _LeftServo.write(leftServoValue);     // sets the servo position according to the scaled value (0 ... 180)
  _RightServo.write(rightServoValue);     // sets the servo position according to the scaled value (0 ... 180)
  //_LeftServo.write(100);     // sets the servo position according to the scaled value (0 ... 180)
  //_RightServo.write(100);     // sets the servo position according to the scaled value (0 ... 180)
}


void ReadSerial()
{
  while (Serial.available())
  {
    _Messenger.process(Serial.read());
    //delayMicroseconds(100);
  }
}

// Define messenger function
void OnMssageCompleted()
{
  //Serial.println("Message completed.");
  if (_Messenger.checkString("s"))
  {
    SetSpeed();
    ClearOutMessenger();
    return;
  }

  if (_Messenger.checkString("reset")) // reset odometry, speed controller, etc.
  {
    ClearOutMessenger();
    Reset();
    return;
  }

  if (_Messenger.checkString("dg")) // DriveGeometry
  {
    InitializeDriveGeometry();
    ClearOutMessenger();
    return;
  }

  if (_Messenger.checkString("sc")) // SpeedControllerParams
  {
    InitializeSpeedControllerParams();
    ClearOutMessenger();
    return;
  }
  
  if (_Messenger.checkString("bm")) // BatteryMonitorParams
  {
    InitializeBatteryMonitor();
  }

  // clear out unrecognized content
  ClearOutMessenger();
}

void ClearOutMessenger()
{
  //Serial.println("Clearing rest of message.");
  //_Messenger.reset();
  while (_Messenger.available())
  {
    // consume complete message
    _Messenger.readChar();
    //Serial.println(_Messenger.readChar());
  }
}

void SetSpeed()
{
  if (!_Messenger.available())
  {
    return;
  }
  float commandedVelocity = _Messenger.readFloat();
  if (!_Messenger.available())
  {
    return;
  }
  float commandedAngularVelocity = _Messenger.readFloat();
  _SpeedController.CommandVelocity(commandedVelocity, commandedAngularVelocity); 
}

void Reset()
{
  _IsInitialized = false;
  _LeftEncoderTicks = 0;
  _RightEncoderTicks = 0;

  _RobotParams.Reset();
  _OdometricLocalizer.Reset();
  _SpeedController.Reset();
  _BatteryMonitor.Reset();
  
  Serial.print("reset_done");
  Serial.print("\n");
}

// set robot params wheel diameter [m], trackwidth [m], ticks per revolution
void InitializeDriveGeometry()
{
  float wheelDiameter = _Messenger.readFloat();
  float trackWidth = _Messenger.readFloat();
  int countsPerRevolution = _Messenger.readInt();
  
  _RobotParams.Initialize(wheelDiameter, trackWidth, countsPerRevolution);

  Serial.print("robotparams");
  Serial.print("\t");
  Serial.print(_RobotParams.WheelDiameter);
  Serial.print("\t");
  Serial.print(_RobotParams.TrackWidth);
  Serial.print("\t");
  Serial.print(_RobotParams.CountsPerRevolution);
  Serial.print("\n");
}

void InitializeSpeedControllerParams()
{
  //Serial.println("Initializing speed controller params.");
  float velocityPParam = _Messenger.readFloat();
  float velocityIParam = _Messenger.readFloat();
  float turnPParam = _Messenger.readFloat();
  float turnIParam = _Messenger.readFloat();
  float commandTimeout = _Messenger.readFloat();
  
  _SpeedController.Initialize(velocityPParam, velocityIParam, turnPParam, turnIParam, commandTimeout);

  Serial.print("PID Params: ");
  Serial.print(velocityPParam);
  Serial.print("\t");
  Serial.print(velocityIParam);
  Serial.print("\t");
  Serial.print(turnPParam);
  Serial.print("\t");
  Serial.print(turnIParam);
  Serial.print("\n");
  /*
  */
}

void InitializeBatteryMonitor()
{
  float voltageTooLowlimit = _Messenger.readFloat();
  _BatteryMonitor.InitializeLowVoltageLimit(voltageTooLowlimit);

  Serial.print("battery monitor Params: ");
  Serial.print(voltageTooLowlimit);
  Serial.print("\n");
  /*
  */
}

float GetFloatFromBaseAndExponent(int base, int exponent)
{
  return base * pow(10, exponent);
}

long mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Interrupt service routines for the left motor's quadrature encoder
void HandleLeftMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _LeftEncoderBSet = digitalRead(c_LeftEncoderPinB);   // read the input pin

  // and adjust counter + if A leads B
  #ifdef LeftEncoderIsReversed
    _LeftEncoderTicks -= _LeftEncoderBSet ? -1 : +1;
  #else
    _LeftEncoderTicks += _LeftEncoderBSet ? -1 : +1;
  #endif
}

// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{
  // Test transition; since the interrupt will only fire on 'rising' we don't need to read pin A
  _RightEncoderBSet = digitalRead(c_RightEncoderPinB);   // read the input pin

  // and adjust counter + if A leads B
  #ifdef RightEncoderIsReversed
    _RightEncoderTicks -= _RightEncoderBSet ? -1 : +1;
  #else
    _RightEncoderTicks += _RightEncoderBSet ? -1 : +1;
  #endif
}

