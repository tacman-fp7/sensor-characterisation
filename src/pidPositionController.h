#pragma once
#include "pidController.h"

class OmegaPositionController: public PidController
{
public:
	virtual double update(double ftValue)
	{
		return (PidController::pidUpdate(ftValue));
	};

	virtual void setSetpoint(double setpoint)
	{
		_rampSetpoint = setpoint;
		_setpoint = setpoint;
		_integral = 0;
		
	}
	virtual void setRampSetpoint(double setpoint)
	{
		
		_rampSetpoint = setpoint;
		_rampValue = (_setpoint - _rampSetpoint)/100;
	   _integral = 0;
	}
};

