#pragma once

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
#include <iostream>
#include "drdc.h"

#define OMEGA_UPPER_LIMIT 0.08 //TODO: set them as class variables
#define OMEGA_LOWER_LIMIT -0.03

using namespace yarp::os;
using namespace std;
bool init_omega(); // routine to initialise omega haptic device

// Storage for omegaData with accessors and mutators
struct omegaData
{
public:
	omegaData():_x(0), _y(0), _z(0), _xBias(0), _yBias(0), _zBias(0){};

	void updateData()
	{

		double robotPose[DHD_MAX_DOF];
		drdGetPositionAndOrientation( robotPose, NULL); // I am not interested in the orientation matrix
		this->setAxesPos(robotPose[0], robotPose[1], robotPose[2] );

	}

	void setZ(double z)
	{
		_mutex.lock();

		if(_z > OMEGA_UPPER_LIMIT)
			_z = OMEGA_UPPER_LIMIT;
		if(_z < OMEGA_LOWER_LIMIT)
			_z = OMEGA_LOWER_LIMIT;
		else
			_z = z;
		_mutex.unlock();
	}

	void setAxesPos(double x, double y, double z)
	{
		_mutex.lock();
		_x = x; _y = y;


		if(_z > OMEGA_UPPER_LIMIT)
			_z = OMEGA_UPPER_LIMIT;
		if(_z < OMEGA_LOWER_LIMIT)
			_z = OMEGA_LOWER_LIMIT;
		else
			_z = z;


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

	Mutex _mutex;
};


struct forceTorqueData
{
	forceTorqueData(): _fx(0), _fy(0), _fz(0), _fxBias(0), _fyBias(0), _fzBias(0){};
	bool updateData(Bottle *ft_input)
	{
		if(ft_input == NULL)
		{
			cout << "Warning! No force/torque data." << endl;
			return false; 
		}

		_fx = ft_input->get(0).asDouble();
		_fy = ft_input->get(1).asDouble();
		_fz = ft_input->get(2).asDouble();
		_tx = ft_input->get(3).asDouble();
		_ty = ft_input->get(4).asDouble();
		_tz = ft_input->get(5).asDouble();
		return true;

	}
	void setForces(double fx, double fy, double fz)
	{
		_mutex.lock();
		_fx = fx; _fy = fy; _fz = fz;
		_mutex.unlock();
	}
	void setTorques(double tx, double ty, double tz)
	{
		_mutex.lock();
		_tx = tx; _ty = ty; _tz = tz;
		_mutex.unlock();
	}
	void setBias()
	{
		_mutex.lock();
		_fxBias = _fx; _fyBias = _fy; _fzBias = _fz;
		_mutex.unlock();
	}
	void setBias(double fxBias, double fyBias, double fzBias)
	{
		_mutex.lock();
		_fxBias = fxBias; _fyBias = fyBias; _fzBias = fzBias;
		_mutex.unlock();
	}

	void getForces(double *fx, double *fy, double *fz)
	{
		_mutex.lock();
		*fx = _fx; *fy = _fy; *fz = _fz;
		_mutex.unlock();

	}
	void getBiasedForces(double *fx, double *fy, double *fz)
	{
		_mutex.lock();
		*fx = _fx - _fxBias; *fy = _fy - _fyBias; *fz = _fz - _fzBias;
		_mutex.unlock();

	}
private:
	double _fx, _fy, _fz, _tx, _ty, _tz;
	double _fxBias, _fyBias, _fzBias, _txBias, _tyBias, _tzBias;
	Mutex _mutex;

};

class OmegaATIThread: public RateThread
{
public:
	OmegaATIThread(int period):RateThread(period){};
	void UpdateOmegaPosition();
	void stepUp();
	void stepDown();
	void updateBias();
	bool threadInit();
	void threadRelease();
	void run();

private:


private:
	BufferedPort<Bottle> _port_ft;
	BufferedPort<Bottle> _port_fingertip;

	//omegaData _omegaData_init; // Storage for initial position

	omegaData _omegaData;
	forceTorqueData _forceTorqueData;
	double _stepSize;
};

