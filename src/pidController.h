#pragma once
#include <cstddef>
#include <time.h>

class PidController
{

public:
	PidController();
	virtual double update(double omegaPosition, double omegaVelocity, double ftValue){return 0;};
	void setKp(double Kp);
	void setKi(double Ki);
	void setKd(double Kd);
	void setOutMax(double outMax);
	void setOutMin(double outMin);
	void setSetpoint(double setpoint);
	double getSetpoint();


	double update(double curVal);
	double update(double curVal, double velocity);

private:
	inline double P(double error);
	inline double I(double error);
	inline double D(double error);

protected:
	double _setpoint;

private:
	double _Kp;
	double _Ki;
	double _Kd;
	double _outMax;
	double _outMin;
	clock_t _clkTicks;
	time_t _timer;
	double _tSample;
	double _integral;
	
};

