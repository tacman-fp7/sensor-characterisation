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
		//_setpoint = 0;
		_rampSetpoint = setpoint;
	   _integral = 0;
	}
};

