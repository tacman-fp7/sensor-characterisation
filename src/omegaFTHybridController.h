#pragma once
#include "pidController.h"

class OmegaFTHybridController: public PidController
{
public:
	virtual double update(double omegaPosition, double omegaVelocity, double ftValue)
	{
		
		PidController::rampSetpoint();
		return (_setpoint + PidController::pidUpdate(ftValue));
	};

	virtual void setRampSetpoint(double setpoint)
	{
		
		
		_rampSetpoint = setpoint;
		_rampValue = (_rampSetpoint - _setpoint)/ 100;// 10000; <-- Was for hysteresis
													  // 100 for normal
	   //_integral = 0;
	}
};

