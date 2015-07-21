#include "pidController.h"
#include <stdio.h>
#include <time.h>
#include <drdc.h>

PidController::PidController()
{
	_Kp = 0;
	_Ki = 0;
	_Kd = 0;
	_setpoint = 0;
	_rampSetpoint = 0;
	_clkTicks = 0;
	_timer = 0;
	_tSample = 1;
	_integral = 0;
	_rampValue = 0;
}

void PidController::InitController(pidParams_t params)
{
	_Kp = params.Kp;
	_Ki = params.Ki;
	_Kd = params.Kd;
	_outMax = params.outMax;
	_outMin = params.outMin;
}
void PidController::setKp(double Kp)
{
	_Kp = Kp;
}
void PidController::setKi(double Ki)
{
	_Ki = Ki;
} 
void PidController::setKd(double Kd)
{
	_Kd = Kd;
}

double PidController::getSetpoint()
{
	return _setpoint;
}
void PidController::setSetpoint(double setpoint)
{
	_setpoint = setpoint;
	_rampSetpoint = setpoint;
	_integral = 0;
}

void PidController::setRampSetpoint(double setpoint)
	{
		_rampSetpoint = setpoint;
		_rampValue = (_setpoint - _rampSetpoint)/100;
	   //_integral = 0;
	}



void PidController::rampSetpoint()
{
	
	

	if(_rampValue > 0){
		if(_setpoint >= _rampSetpoint)
		{
			//printf("Ramp reached % 1.3f\n", _integral);
			return;
		}
	}
	else
	{

		if(_setpoint <= _rampSetpoint)
		{
			//printf("Ramp reached % 1.3f\n", _integral);
			return;
		}
	}

	
	_setpoint += _rampValue;
	printf("%0.2f\r", _setpoint);
	//_integral = 0;

	
	
	
}

inline double PidController::P(double error)
{
	//printf("Error: % 3.3f, Setpoint: % 3.3f\n", error, _setpoint);
	return (_Kp * error);
}

void PidController::setOutMax(double outMax)
{
	_outMax = outMax;
}

inline double PidController::I(double error)
{
	
	_integral += (error  * _Ki); // TODO: add _tSample
	
	if(_integral < _outMin/2)
		_integral = _outMin/2;
	else if(_integral > _outMax/2)
		_integral = _outMax/2;
	
	return (_integral);
}

inline double PidController::D(double error)
{
	static double prevError = error; // TODO: change this to a non-static variable
	double ret = (error - prevError) * _Kd; //Assuming contstant intervals
	prevError = error;
	return (ret / _tSample);
	
}

void PidController::setOutMin(double outMin)
{
	_outMin = outMin;
}


double PidController::pidUpdate(double curVal)
{

	_tSample = 1; //TODO: fix this later to actual number of milliseconds

	double error = _setpoint - curVal;
	double ret;

	double myP = P(error);
	double i = I(error);
	double d = D(error);
	
	
		ret = myP +  i + d; //D(error);
	
	if(ret < _outMin)
		ret = _outMin;
	else if(ret > _outMax)
		ret = _outMax;

	//printf("Error: % 3.3f, Setpoint: % 3.3f, FT: % 3.3f :: ", error, _setpoint, curVal);
	//printf("P: % 3.3f, I: % 3.3f, % 3.3f = % 3.3f\n", myP, i, d, ret); 
	return ret;
}


double PidController::update(double curVal, double velocity)
{

	_tSample = 1; //TODO: fix this later to actual number of milliseconds

	double error = _setpoint - curVal;
	double ret;

	double myP = P(error);
	double i = I(error);
	double d = -1 * velocity * _Kd;
	
	
		ret = myP +  i + d; //D(error);
	
	if(ret < _outMin)
		ret = _outMin;
	else if(ret > _outMax)
		ret = _outMax;

	//printf("Error: % 3.3f, Setpoint: % 3.3f, FT: % 3.3f :: ", error, _setpoint, curVal);
	//printf("P: % 3.3f, I: % 3.3f, % 3.3f = % 3.3f\n", myP, i, d, ret); 
	return ret;
}