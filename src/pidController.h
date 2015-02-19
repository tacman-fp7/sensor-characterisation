#pragma once
#include <cstddef>
#include <time.h>

class PidController
{

public:
	PidController();
	void setKp(double Kp);
	void setKi(double Ki);
	void setKd(double Kd);
	void setOutMax(double outMax);
	void setOutMin(double outMin);
	void setSetpoint(double setpoint);
	double update(double curVal);

private:
	inline double P(double error);
	inline double I(double error);
	inline double D(double error);

private:
	double _Kp;
	double _Ki;
	double _Kd;
	double _setpoint;
	double _outMax;
	double _outMin;
	clock_t _clkTicks;
	time_t _timer;
	double _tSample;
	
};

