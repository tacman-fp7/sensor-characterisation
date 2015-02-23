#pragma once

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
//#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
//#include <yarp/dev/ControlBoardPid.h>
#include <iostream>
#include "drdc.h"
#include <queue>
#include <array>
#include "pidController.h"
#include "forceTorqueData.h"
#include <ctime>

#define FILTER_OFF
#define USE_FORCE_CONTROLLER
//#define USE_POSITION_CONTROLLER

//TODO: Gotta clean it and put it in a config file
#define OMEGA_UPPER_LIMIT 0.07 //TODO: set them as class variables
#define OMEGA_LOWER_LIMIT -0.035
#define OMEGA_Z_MIN -0.035
#define OMEGA_Z_MAX 0.07
#define OMEGA_Z_SETPOINT -0.5

#ifdef FILTER_ON
#define OMEGA_Z_KP 4 * pow(10, -6)
#define OMEGA_Z_KI 2 * pow(10,-11)
#define OMEGA_Z_KD 1 * pow(10,-4)
#endif // FILTER_ON



#ifdef FILTER_OFF
#define OMEGA_Z_KP 9 * pow(10, -6)
#define OMEGA_Z_KI 2 * pow(10,-11)
#define OMEGA_Z_KD 1 * pow(10,-4)
#endif // FILTER_OFF

#define FORCE_KP 1000
#define FORCE_KI 20
#define FORCE_KD 20 
#define OMEGA_FORCE_MAX 12

#define FT_FORCE_KP 0.5
#define FT_FORCE_KI 0.005
#define FT_FORCE_KD 0.01
#define FT_FORCE_MAX 8
#define FT_SETPOINT -4



#define FILTER_WINDOW 10
#define FT_CHANNELS 6

//typedef struct forceTorqueData forceTorqueDataType;

using namespace yarp::os;
using namespace std;
bool init_omega_dhd(); // routine to initialise omega haptic device
bool init_omega_drd();

struct omegaForce
{
	omegaForce(): _fx(0), _fy(0), _fz(0){};

	void setForces(double fx, double fy, double fz)
	{
		_fx = fx;
		_fy = fy;
		fz = fz;
	}
	void getFroces(double* fx, double* fy, double* fz)
	{
		*fx = _fx;
		*fy = _fy;
		*fz = _fz;
	}
private:
	double _fx;
	double _fy;
	double _fz;
};
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





class OmegaATIThread: public RateThread
{
public:
	OmegaATIThread(int period):RateThread(period){};
	void UpdateOmegaPosition();
	void stepUp();
	void stepDownTest();
	void stepDown();
	void updateBias();
	bool threadInit();
	void threadRelease();
	void run();

private:

	void PositionControl();
	void ForceControl();

private:
	BufferedPort<Bottle> _port_ft;
	BufferedPort<Bottle> _port_fingertip;
	BufferedPort<Bottle> _port_omega;
	BufferedPort<Bottle> _port_ftFiltered;

	yarp::os::Stamp _timeStamp;

	//omegaData _omegaData_init; // Storage for initial position

	omegaData _omegaData;
	omegaForce _omegaForce;

	forceTorqueData _forceTorqueData;
	PidController _zController;

	PidController _xForceController;
	PidController _yForceController;
	PidController _zForceController;

	PidController _zOmegaFTController;

	bool _ftNotBiased;
	double _stepSize;
	std::clock_t _time;
};

