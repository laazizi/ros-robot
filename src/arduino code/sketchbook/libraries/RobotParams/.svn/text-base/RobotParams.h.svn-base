/*
  RobotParams.h - A very simple class that is used to pass around robot specific parameters

  Dr. Rainer Hessmer, Decmber, 2010.
  Released into the public domain.
*/

#ifndef RobotParams_h
#define RobotParams_h

#include "WProgram.h"

#define PI 3.14159265

class RobotParams
{
	public:
		bool IsInitialized;
		double WheelDiameter;
		double TrackWidth;
		double CountsPerRevolution;
		double DistancePerCount;
		double RadiansPerCount;


		RobotParams()
		{
			IsInitialized = false;
		}

		// initializes robot params wheel diameter [m], trackwidth [m], ticks per revolution
		void Initialize(double wheelDiameter, double trackWidth, int countsPerRevolution)
		{
			WheelDiameter = wheelDiameter;
			TrackWidth = trackWidth;
			CountsPerRevolution = countsPerRevolution;

			DistancePerCount = (PI * wheelDiameter) / (double)countsPerRevolution;
			RadiansPerCount = DistancePerCount / trackWidth;

			IsInitialized = true;
		}
};

#endif
