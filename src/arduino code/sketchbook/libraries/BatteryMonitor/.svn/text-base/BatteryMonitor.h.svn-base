/*
  BatteryMonitor.h - Battery monitor library.
  Dr. Rainer Hessmer, January, 2011.
  Released into the public domain.
*/

#ifndef BatteryMonitor_h
#define BatteryMonitor_h

#include "WProgram.h"


class BatteryMonitor
{
private:
	int _ScaledBatteryVInPin;
	float _VInToVBatteryRatio;
	float _VoltageTooLowLimit;

public:
	bool IsInitialized;
	float BatteryVoltage;
	bool VoltageIsTooLow;

	// scaledBatteryVInPin: The analog input pin from which to read the scaled battery voltage.
	// Typically the battery voltage is higher than the 5V the Arduino board can measure and a
	// voltage devider is used to scale the battery voltage to a range that can be measured by Arduino.
	// vInToVBatteryRatio: Ratio between the voltage measured at the input pin and the actual battery
	// voltage.
	BatteryMonitor(int scaledBatteryVInPin, float vInToVBatteryRatio)
	{
		_ScaledBatteryVInPin = scaledBatteryVInPin;
		_VInToVBatteryRatio = vInToVBatteryRatio;

		IsInitialized = false;
	}

	void InitializeLowVoltageLimit(float voltageTooLowlimit)
	{
		if (voltageTooLowlimit > 0.0)
		{
			_VoltageTooLowLimit = voltageTooLowlimit;
			IsInitialized = true;
		}
	}

	void Update()
	{
		int rawValue = analogRead(_ScaledBatteryVInPin);         // read the voltage on the divider
  		float scaledBatteryVoltage = rawValue * 0.00488;   // A reading of 1 for the A/D = 5V / 1024 = 0.00488 V
		BatteryVoltage = scaledBatteryVoltage * _VInToVBatteryRatio;

		if (BatteryVoltage > _VoltageTooLowLimit)
		{
			VoltageIsTooLow = false;
		}
		else
		{
			VoltageIsTooLow = true;
		}
	}
};

#endif
