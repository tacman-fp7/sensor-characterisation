#include "omegaData.h"


void OmegaData::publishData(yarp::os::Stamp& timeStamp)
{
	Bottle& omegaOutput = _port_omega.prepare();
	omegaOutput.clear();

	double 	drdPos[DHD_MAX_DOF];
	drdGetPositionAndOrientation(drdPos, NULL);
	for(int i = 0; i < 3; i++)
		omegaOutput.addDouble(drdPos[i]);

	_mutex.lock();
	//_timeStamp.update();
	_port_omega.setEnvelope(timeStamp);
	_mutex.unlock();

	_port_omega.writeStrict();
}