#pragma once
#include "pidController.h"

class OmegaForceController: public PidController
{
public:
	double update(double omegaPosition, double omegaVelocity, double ftValue)
	{
		PidController::rampSetpoint();
		return PidController::update(omegaPosition, omegaVelocity);
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