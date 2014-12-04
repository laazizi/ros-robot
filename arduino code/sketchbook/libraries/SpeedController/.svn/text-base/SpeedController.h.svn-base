/*
  SpeedController.h - Contains the PID controllers to control the speed of the robot

  Dr. Rainer Hessmer,December, 2010.
  Released into the public domain.
*/

#ifndef SpeedController_h
#define SpeedController_h

#include "OdometricLocalizer.h"
#include "RobotParams.h"
#include "TimeInfo.h"
#include "WProgram.h"

class SpeedController
{
private:
	float _PParam, _IParam, _DParam;
	float _CommandTimeout;
	long _MillisecsLastCommand;
	OdometricLocalizer* _pOdometricLocalizer;

	RobotParams* _pRobotParams;
	TimeInfo* _pTimeInfo;

	//float _LastLeftError, _LastRightError, _LastTurnError;
	float _LeftErrorIntegral, _RightErrorIntegral, _TurnErrorIntegral;
	
	float _MaxVelocityErrorIntegral;
	float _MaxTurnErrorIntegral;

	float ClipIntegralContribution(float integralContribution)
	{
		if (integralContribution > 1.0)
		{
			return 1.0;
		}
		else if (integralContribution < -1.0)
		{
			return -1.0;
		}
	}
	
	void Reset()
	{
		//_LastLeftError = 0.0;
		//_LastRightError = 0.0;
		_LeftErrorIntegral = 0.0;
		_RightErrorIntegral = 0.0;
		_TurnErrorIntegral = 0.0;
		LeftError = 0.0;
		RightError = 0.0;	
		TurnError = 0.0;	
		NormalizedLeftCV = 0.0;
		NormalizedRightCV = 0.0;

		CommandedVelocity = 0.0;
		CommandedAngularVelocity = 0.0;
	}
	
public:
	bool IsInitialized;

	float VelocityPParam;
	float VelocityIParam;
	//float VelocityDParam;

	float TurnPParam;
	float TurnIParam;

	float CommandedVelocity;
	float CommandedAngularVelocity;

	float LeftError;
	float RightError;
	float TurnError;

	// normalized control values for the left and right motor
	float NormalizedLeftCV;
	float NormalizedRightCV;

	SpeedController(OdometricLocalizer* pOdometricLocalizer, RobotParams* pRobotParams, TimeInfo* pTimeInfo)
	{
		_pOdometricLocalizer = pOdometricLocalizer;

		_pRobotParams = pRobotParams;
		_pTimeInfo = pTimeInfo;
		
		IsInitialized = false;

		VelocityPParam = 0.0;
		VelocityIParam = 0.0;
		
		TurnPParam = 0.0;
		TurnIParam = 0.0;

		CommandedVelocity = 0.0;
		CommandedAngularVelocity = 0.0;
	}
	
	void Initialize(float velocityPParam, float velocityIParam, float turnPParam, float turnIParam, float commandTimeout)
	{
		VelocityPParam = velocityPParam;
		VelocityIParam = velocityIParam;

		TurnPParam = turnPParam;
		TurnIParam = turnIParam;
		
		// Avoiding runaway integral error contributions
		_MaxVelocityErrorIntegral = 1 / VelocityIParam;
		_MaxTurnErrorIntegral = 1 / TurnIParam;
		
		_CommandTimeout = commandTimeout;
		_MillisecsLastCommand = millis();

		IsInitialized = true;
	}
	
	void CommandVelocity(float commandedVelocity, float commandedAngularVelocity)
	{
		CommandedVelocity = commandedVelocity;
		CommandedAngularVelocity = commandedAngularVelocity;
		_MillisecsLastCommand = millis();
	}
	
	void Update(bool batteryVoltageIsTooLow)
	{
		if (batteryVoltageIsTooLow)
		{
			// we need to stop the motors and stop accumulating errors
			Reset();
			return;
		}

		// How long has it been since we received the last command
		float secondsSinceLastCommand = (millis() - _MillisecsLastCommand) / 1000.0;
		if (secondsSinceLastCommand > _CommandTimeout)
		{
			// we need to stop the motors and stop accumulating errors
			Reset();
			return;
		}

		float angularVelocityOffset = 0.5 * CommandedAngularVelocity * _pRobotParams->TrackWidth;
		
		float expectedLeftSpeed = CommandedVelocity - angularVelocityOffset;
		float expectedRightSpeed = CommandedVelocity + angularVelocityOffset;

		LeftError = expectedLeftSpeed - _pOdometricLocalizer->VLeft;
		RightError = expectedRightSpeed - _pOdometricLocalizer->VRight;
		TurnError = LeftError - RightError; // if > 0 then we need to steer more to the left (and vice versa)

		_LeftErrorIntegral += LeftError;
		_RightErrorIntegral += RightError;
		_TurnErrorIntegral += TurnError;

		// Avoiding runaway integral error contributions
		_LeftErrorIntegral = constrain(_LeftErrorIntegral, -_MaxVelocityErrorIntegral, +_MaxVelocityErrorIntegral);
		_RightErrorIntegral = constrain(_RightErrorIntegral, -_MaxVelocityErrorIntegral, +_MaxVelocityErrorIntegral);
		_TurnErrorIntegral = constrain(_TurnErrorIntegral, -_MaxTurnErrorIntegral, +_MaxTurnErrorIntegral);

		//float leftErrorDifferential = (LeftError - _LastLeftError);
		//float rightErrorDifferential = (RightError - _LastRightError);
		
		NormalizedLeftCV = VelocityPParam * LeftError + VelocityIParam * _LeftErrorIntegral
		                   + (TurnPParam * TurnError + TurnIParam * _TurnErrorIntegral);
		NormalizedLeftCV = constrain(NormalizedLeftCV, -1, +1);

		NormalizedRightCV = VelocityPParam * RightError + VelocityIParam * _RightErrorIntegral;
		                    - (TurnPParam * TurnError + TurnIParam * _TurnErrorIntegral);
		NormalizedRightCV = constrain(NormalizedRightCV, -1, +1);
		
		//_LastLeftError = LeftError;
		//_LastRightError = RightError;
	}
};

#endif
