#pragma once
#include "pidController.h"

class OmegaForceController: public PidController
{
	double update(double omegaPosition, double omegaVelocity, double ftValue)
	{
		return PidController::update(omegaPosition, omegaVelocity);
	};
};