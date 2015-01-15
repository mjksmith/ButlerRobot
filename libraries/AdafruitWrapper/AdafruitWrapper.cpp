// ADAFRUITWRAPPER.CPP
#include "Arduino.h"
#include "AFMotor.h"
#include "AdafruitWrapper.h"

AdafruitWrapper::AdafruitWrapper(int leftMotorPort, int rightMotorPort,
	int minSpeed, int maxSpeed) {
	int leftMotorFreq, rightMotorFreq;
	leftMotorFreq = rightMotorFreq = MOTOR12_1KHZ;
	
	if(leftMotorPort==3 || leftMotorPort==4)
		leftMotorFreq = MOTOR34_1KHZ;
	if(rightMotorPort==3 || rightMotorPort==4)
		rightMotorFreq = MOTOR34_1KHZ;
		
	
	_leftMotor = new AF_DCMotor(leftMotorPort, leftMotorFreq);
	_rightMotor = new AF_DCMotor(rightMotorPort, rightMotorFreq);
	
	_minSpeed = minSpeed;
	_maxSpeed = maxSpeed;
}

/*
// This function communicates with the Adafruit Motor Controller's
proprietary motor controller library to adjust the duty cycle of the signal
powering the motors. This function also manipulates the provided speed by
scaling and offsetting it to help overcome motor stiction.
(Written by Justin Lim)
*/
void AdafruitWrapper::set(int speed) {
	speed *= (_maxSpeed - _minSpeed);
	speed /= _maxSpeed;
	
	if(speed > 0) {	
		speed += _minSpeed;
	} else if(speed < 0) {
		speed -= _minSpeed;
	}
	
	_leftMotor->setSpeed(abs(speed));
	_rightMotor->setSpeed(abs(speed));
  	if(speed > 0) {
    	_leftMotor->run(FORWARD);
    	_rightMotor->run(FORWARD);
  	} else {
    	_leftMotor->run(BACKWARD);
		_rightMotor->run(BACKWARD);
	}
}

void AdafruitWrapper::cleanUp() {
	delete _leftMotor;
	delete _rightMotor;
}
