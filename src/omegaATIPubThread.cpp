#include "omegaATIPubThread.h"
#include <iostream>

void OmegaATIPubThread::run()
{
	// Save the data.

	if(_forceTorqueData == NULL)
		return; //TODO: printout a warning
	if(_omegaData == NULL)
		return; //TODO: printout a warning

	_sem_publishData.wait();
	_forceTorqueData->publishData();
	_omegaData->publishData();

	

}

bool OmegaATIPubThread::threadInit()
{
	if(_omegaData == NULL)
		return false;

	if(_forceTorqueData == NULL)
		return false;

	return true;
}

void OmegaATIPubThread::threadRelease()
{

}

void OmegaATIPubThread::publishData()
{
	_sem_publishData.post();
}