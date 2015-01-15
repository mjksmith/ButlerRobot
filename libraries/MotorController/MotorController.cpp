// MOTORCONTROLLER.CPP
#include "MotorController.h"
#include "Arduino.h"

MotorController::MotorController(int pinA, int pinB, int pinFwdA, int pinRevA,
	 int pinFwdB, int pinRevB, int minSpeed, int maxSpeed) {
	_pinA = pinA;
	_pinB = pinB;
	
	_pinFwdA = pinFwdA;
	_pinRevA = pinRevA;
	_pinFwdB = pinFwdB;
	_pinRevB = pinRevB;
	
	_minSpeed = minSpeed;
	_maxSpeed = maxSpeed;
	
	pinMode(_pinA, OUTPUT);
	pinMode(_pinB, OUTPUT);
	pinMode(_pinFwdA, OUTPUT);
	pinMode(_pinRevA, OUTPUT);
	pinMode(_pinFwdB, OUTPUT);
	pinMode(_pinRevB, OUTPUT);

	_last = 0;
	_delay = 0;
}

void MotorController::set(int speed) {
	if(speed > 0) {
		fwdA();
		fwdB();
	} else {
		revA();
		revB();
	}
	
	speed *= (_maxSpeed - _minSpeed);
	speed /= _maxSpeed;
	
	if(speed > 0) {	
		speed += _minSpeed;
	} else if(speed < 0) {
		speed -= _minSpeed;
	}
	
	analogWrite(_pinA, speed);
	analogWrite(_pinB, speed);
}

void MotorController::fwdA() {
	digitalWrite(_pinFwdA, HIGH);
	digitalWrite(_pinRevA, LOW);
}
		
void MotorController::revA() {
	digitalWrite(_pinFwdA, LOW);
	digitalWrite(_pinRevA, HIGH);
}
		
void MotorController::fwdB() {
	digitalWrite(_pinFwdB, HIGH);
	digitalWrite(_pinRevB, LOW);
}
		
void MotorController::revB() {
	digitalWrite(_pinFwdB, LOW);
	digitalWrite(_pinRevB, HIGH);
}
