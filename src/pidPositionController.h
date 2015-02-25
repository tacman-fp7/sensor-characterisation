#pragma once
#include "pidController.h"

class OmegaPositionController: public PidController
{
public:
	virtual double update(double ftValue)
	{
		return (PidController::pidUpdate(ftValue));
	};
};

