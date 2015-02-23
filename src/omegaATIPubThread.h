#pragma once
#include <yarp/os/RateThread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>

using namespace yarp::os;
using namespace std;

class OmegaATIPubThread: public RateThread
{

public:
	OmegaATIPubThread(int period):RateThread(period){};
    bool threadInit();
	void threadRelease();
	void run();
private:
	Semaphore _writeData;

};