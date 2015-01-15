// PID.CPP
#include "Arduino.h"
#include <PID.h>

PID::PID(double Kp, double Ki, double Kd, double & setpoint) {
    _maxSpeed = 100;
    _integralTerm = 0.0;
    _samplingPeriod = 10;
    
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
	
	_lastUpdate = millis()-_samplingPeriod;
    _setpoint = setpoint;
    _freshData = false;
}

double PID::returnOutput(double input) {
	double timeDiff = (millis() - _lastUpdate);
   
	if(timeDiff >= _samplingPeriod) { // samplingPeriod = the period of the IMU
   		_lastUpdate = millis();
   		_freshData = true; // let the motor controller know the PID has data
   		_lastError = input;
   		
   		timeDiff /= 1000; //convert ms into seconds
      
		double error = _setpoint - input;
		_integralTerm += error*timeDiff;
		double derivative = (input - _lastError)/timeDiff;
   		double output = _Kp * error + _Ki * _integralTerm + _Kd * derivative;
      
		if(abs(output) > _maxSpeed)
	 		output = output * _maxSpeed / abs(output);
		return output;
	} else {
		_freshData = false;
		return 0.0;
	}
}

void PID::setMaxSpeed(double max) {
	_maxSpeed = max;
}
void PID::incP(double inc) {
	_Kp += inc;
}
void PID::incI(double inc) {
	_Ki += inc;
}
void PID::incD(double inc) {
	_Kd += inc;
}

bool PID::freshData() {
	return _freshData;
}

void PID::setSetpoint(double setpoint) {
	_setpoint = setpoint;
}
