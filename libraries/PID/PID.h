// PID.H
#ifndef PID_h
#define PID_h

class PID
{
  private:
    double _setpoint;
	int _samplingPeriod;
	double _Kp;
    double _Ki;
    double _Kd;
	bool _freshData;
	
	unsigned long _lastUpdate;
	double _integralTerm, _lastError;
	double _maxSpeed;
	
  public:
    PID(double Kp, double Ki, double Kd, double & setpoint);
	double returnOutput(double input);
    void setMaxSpeed(double max);
    void setSetpoint(double setpoint);
    void incP(double inc);
    void incI(double inc);
    void incD(double inc);
    bool freshData();
};
#endif

