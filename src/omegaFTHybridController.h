#pragma once
#include "pidController.h"

class OmegaFTHybridController: public PidController
{
	virtual double update(double omegaPosition, double omegaVelocity, double ftValue)
	{
		return (_setpoint + PidController::update(ftValue));
	};
};

