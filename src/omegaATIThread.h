#pragma once

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>

using namespace yarp::os;

bool init_omega(); // routine to initialise omega haptic device

// Storage for omegaData with accessors and mutators
struct omegaData
{
public:
	omegaData():_x(0), _y(0), _z(0){};

	void setZ(double z)
	{
		_mutex.lock();
		_z = z;
		_mutex.unlock();
	}

	void setAxesPos(double x, double y, double z)
	{
		_mutex.lock();
		_x = x; _y = y; _z = z;
		_mutex.unlock();
	}

	void getAxesPos(double *x, double *y, double *z)
	{	
		_mutex.lock();
		*x = _x; *y = _y; *z = _z;
		_mutex.unlock();
	}
private:
	double _x;
	double _y;
	double _z;


	Mutex _mutex;
};


class OmegaATIThread: public RateThread
{
public:
	OmegaATIThread(int period):RateThread(period){};
	void UpdateOmegaPosition();
	bool threadInit();
	void threadRelease();
	void run();

private:
	

private:
	BufferedPort<Bottle> _port_ft;
	BufferedPort<Bottle> _port_fingertip;
	omegaData _omegaData_init; // Storage for initial position
	omegaData _omegaData; 
};

