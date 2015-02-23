#pragma once

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Mutex.h>
#include <iostream>
#include "drdc.h"
#include <queue>
#include <array>
#include "pidController.h"
#include "forceTorqueData.h"
#include <ctime>

#include "omegaATIPubThread.h"
#include "omegaData.h"
#include "omegaFTHybridController.h"
#include "omegaForceController.h"

#undef FILTER_ON
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
bool init_omega();


class OmegaATIThread: public RateThread
{
public:
	OmegaATIThread(int period):RateThread(period)
	{_omegaATIPubThread = NULL;
	_zController = NULL;
	};
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

	OmegaData _omegaData;


	ForceTorqueData _forceTorqueData;
	PidController _zpController;


	OmegaForceController _xForceController;
	OmegaForceController _yForceController;
	OmegaForceController _zForceController;

	//  PidController _xForceController;
	//	PidController _yForceController;
	//	PidController _zForceController;

	//PidController _zOmegaFTController;
	OmegaFTHybridController _zOmegaFTController;

	// Decide during runtime which subcontroller to choose
	PidController* _xController;
	PidController* _yController;
	PidController* _zController;

	OmegaATIPubThread* _omegaATIPubThread;

	bool _ftNotBiased;
	double _stepSize;
	std::clock_t _time;
};

