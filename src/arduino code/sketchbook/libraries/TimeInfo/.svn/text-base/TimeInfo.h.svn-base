/*
  TimeInfo.h - A very simple class that contains time related information

  Dr. Rainer Hessmer, May, 2010.
  Released into the public domain.
*/

#ifndef TimeInfo_h
#define TimeInfo_h

#include "WProgram.h"

class TimeInfo
{
	public:
		unsigned long LastUpdateMicrosecs;		
		unsigned long LastUpdateMillisecs;
		unsigned long CurrentMicrosecs;
		//unsigned long CurrentMillisecs;
		unsigned long MicrosecsSinceLastUpdate;
		//unsigned long MillisecsSinceLastUpdate;
		float SecondsSinceLastUpdate;

		TimeInfo()
		{
			CurrentMicrosecs = micros();
			LastUpdateMicrosecs = CurrentMicrosecs;
			MicrosecsSinceLastUpdate = 0;
			SecondsSinceLastUpdate = 0.0;
		}

		void Update()
		{
			CurrentMicrosecs = micros();
			LastUpdateMillisecs = millis();
			// this rolls over after roughly 70 minutes at the max ulong value 4,294,967,295
			MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
			if (MicrosecsSinceLastUpdate < 0)
			{
				// rollover occured
				MicrosecsSinceLastUpdate = 4294967295 - LastUpdateMicrosecs + CurrentMicrosecs;
			}
			LastUpdateMicrosecs = CurrentMicrosecs;
			SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;
		}
};

#endif
