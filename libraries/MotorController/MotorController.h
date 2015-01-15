// MOTORCONTROLLER.H
#ifndef MotorController_h
#define MotorController_h

#include "Arduino.h"

class MotorController {
	private:
		int _pinA, _pinB, _pinFwdA, _pinRevA, _pinFwdB, _pinRevB, _last,
			_delay, _minSpeed, _maxSpeed;
	public:
		MotorController(int pinA, int pinB, int pinFwdA, int pinRevA,
		int pinFwdB, int pinRevB, int minSpeed, int maxSpeed);
		void set(int speed);
		void fwdA();
		void revA();
		void fwdB();
		void revB();
};

#endif
