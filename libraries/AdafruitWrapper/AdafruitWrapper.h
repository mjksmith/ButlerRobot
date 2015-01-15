// ADAFRUITWRAPPER.H
#ifndef AdafruitWrapper_h
#define AdafruitWrapper_h

#include "AFMotor.h"

class AdafruitWrapper {
	private:
		AF_DCMotor *_leftMotor, *_rightMotor;
		int _minSpeed, _maxSpeed;
	public:
		AdafruitWrapper(int leftMotorPort, int rightMotorPort,
						int minSpeed, int maxSpeed);
		void set(int speed);
		void cleanUp();
};

#endif
