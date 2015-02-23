#pragma once
#include<drdc.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>

#define OMEGA_UPPER_LIMIT 0.07 //TODO: set them as class variables
#define OMEGA_LOWER_LIMIT -0.035

using namespace yarp::os;
// Storage for omegaData with accessors and mutators
class OmegaData
{
public:
	OmegaData():_x(0), _y(0), _z(0), _xBias(0), _yBias(0), _zBias(0)
	{
		_timeStamp.update();
		_port_omega.open("/OmegaATI/omegaPosition");
	};

	void publishData();

	void updateData()
	{
		/*
	// Sending the current omega Position
	double 	drdPos[DHD_MAX_DOF];
	drdGetPositionAndOrientation(drdPos, NULL);
	Bottle& omegaOutput = _port_omega.prepare();
	omegaOutput.clear();
	for(int i = 0; i < 3; i++)
		omegaOutput.addDouble(drdPos[i]);
	_port_omega.setEnvelope(_timeStamp);
	_port_omega.write();
	//printf("Actual  pos: % 3.4f, % 3.4f, % 3.4f\n\n", drdPos[0], drdPos[1], drdPos[2]);

	*/


		double robotPose[DHD_MAX_DOF];
		drdGetPositionAndOrientation( robotPose, NULL); // I am not interested in the orientation matrix
		this->setAxesPos(robotPose[0], robotPose[1], robotPose[2] );

	}

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

	double getZ(void)
	{
		double ret;
		_mutex.lock();
		ret = _z;
		_mutex.unlock();
		return ret;
	}

	void getAxesPos(double *x, double *y, double *z)
	{	
		_mutex.lock();
		*x = _x; *y = _y; *z = _z;
		_mutex.unlock();
	}
	void getAxesPosBiased(double *x, double *y, double *z)
	{	
		_mutex.lock();
		*x = _x - _xBias; *y = _y - _yBias; *z = _z - _zBias;
		_mutex.unlock();
	}
	void setBias()
	{
		_mutex.lock();
		_xBias = _x; _yBias = _y; _zBias = _z;
		_mutex.unlock();
	}

	void setBias(double x, double y, double z)
	{
		_mutex.lock();
		_xBias = x; _yBias = y; _zBias = z;
		_mutex.unlock();
	}
private:
	double _x;
	double _y;
	double _z;
	double _xBias, _yBias, _zBias;
	BufferedPort<Bottle> _port_omega;
	yarp::os::Mutex _mutex;
	yarp::os::Stamp _timeStamp;
};