/*
  QuadratureEncoder.h - Quadrature encoder library.
  Dr. Rainer Hessmer, April, 2010.
  Released into the public domain.

  Inspired by the code from http://www.arduino.cc/playground/Main/RotaryEncoders
  Incorporates code from mikkoh and others that contributed to the referenced article.
*/

#ifndef QuadratureEncoder_h
#define QuadratureEncoder_h

#include "WProgram.h"

class QuadratureEncoder
{
	/*  
	Wraps the encoder setup and the update functions in a class

	!!! NOTE : User must call the functions OnAChanged and OnBChanged from an
	interrupt function him/herself! 

	// ------------------------------------------------------------------------------------------------
	// Example usage :
	// ------------------------------------------------------------------------------------------------
		#include "QuadratureEncoder.h"

		QuadratureEncoder encoder(2, 3);

		void setup()
		{ 
			attachInterrupt(0, HandleInterruptA, CHANGE); 
			attachInterrupt(1, HandleInterruptB, CHANGE); 
		} 

		void loop()
		{
			// do some stuff here - the joy of interrupts is that they take care of themselves
		}

		void HandleInterruptA()
		{
			encoder.OnAChanged();
		}    

		void HandleInterruptB()
		{
			encoder.OnBChanged();
		}    

		// ------------------------------------------------------------------------------------------------
	// Example usage end
	// ------------------------------------------------------------------------------------------------
	*/


public:
	// pinA and pinB must be one of the external interupt pins
	QuadratureEncoder(int pinA, int pinB, boolean isReversed)
	{
		Initialize(pinA, pinB, pinA, isReversed);
	}

	// pinA and pinB must be one of the external interupt pins
	QuadratureEncoder(int pinA, int pinB, int pinA2, boolean isReversed)
	{
		Initialize(pinA, pinB, pinA2, isReversed);
	}

	long GetPosition () { return _Position; };
	void SetPosition (const long p) { _Position = p; };

	// Interrupt on A changing state
	void OnAChanged()
	{
		// Test transition
		_ASet = digitalRead(_PinA2);
		_BSet = digitalRead(_PinB);
		// and adjust counter + if A leads B
		if (_IsReversed)
			_Position -= (_ASet != _BSet) ? +1 : -1;
		else
			_Position += (_ASet != _BSet) ? +1 : -1;
	}

	// Interrupt on B changing state
	void OnBChanged()
	{
		// Test transition
		_ASet = digitalRead(_PinA2);
		_BSet = digitalRead(_PinB);
		// and adjust counter + if B follows A
		if (_IsReversed)
			_Position -= (_ASet == _BSet) ? +1 : -1;
		else
			_Position += (_ASet == _BSet) ? +1 : -1;	
	}

	void GetInfo(volatile unsigned long& milliSecs, volatile bool& a, volatile bool& b, volatile long& position)
	{
		milliSecs = millis();
		a = _ASet;
		b = _BSet;
		position= _Position;
	}

private:
	int _PinA, _PinA2, _PinB;
	volatile bool _ASet, _BSet;
	boolean _IsReversed;
	volatile long _Position;

	void Initialize(int pinA, int pinB, int pinA2, boolean isReversed)
	{
		_PinA = pinA;
		_PinA2 = pinA2;

		_PinB = pinB;

		pinMode(_PinA, INPUT);      // sets pin A as input
		digitalWrite(_PinA, LOW);  // turn on pullup resistors

		pinMode(_PinA2, INPUT);
		digitalWrite(_PinA2, LOW);  // turn on pullup resistors

		pinMode(_PinB, INPUT);      // sets pin B as input
		digitalWrite(_PinB, LOW);  // turn on pullup resistors

		_ASet = digitalRead(_PinA2);   // read the input pin
		_BSet = digitalRead(_PinB);   // read the input pin

		_IsReversed = isReversed;
		_Position = 0;
	}
};

#endif
