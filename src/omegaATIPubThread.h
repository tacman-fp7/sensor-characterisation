#pragma once
#include <yarp/os/RateThread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>
#include "forceTorqueData.h"
#include "omegaData.h"

using namespace yarp::os;
using namespace std;

class OmegaATIPubThread: public RateThread
{

public:
	OmegaATIPubThread(int period, ForceTorqueData* forceTorqueData, OmegaData* omegaData):RateThread(period),
		_forceTorqueData(forceTorqueData), _omegaData(omegaData){};
    bool threadInit();
	void threadRelease();
	void run();
	void publishData();
private:
	Semaphore _sem_publishData;
	ForceTorqueData* _forceTorqueData;
	OmegaData* _omegaData;
};