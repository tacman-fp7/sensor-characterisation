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

#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/all.h>



using namespace yarp::os;
using namespace std;



class OmegaATIThread: public RateThread
{
	
public:
	OmegaATIThread(int period, ResourceFinder& rsf):RateThread(period), _rsf(rsf)
	{
		_positionControl = false;
	_omegaATIPubThread = NULL;
	_zController = NULL;
	AdjuctControlOutput = NULL;
	_ftZForce = 0;
	memset(&_pidPosCtrl_filterOn, 0, sizeof(pidParams_t)); 
	memset(&_pidPosCtrl_filterOff, 0, sizeof(pidParams_t)); 
	memset(&_pidParams_omegaForceCtrl, 0, sizeof(pidParams_t)); 
	memset(&_pidParams_FTForceCtrl, 0, sizeof(pidParams_t)); 

	};
	void UpdateOmegaPosition();
	void stepUp();
	void stepDownTest();
	void stepDown();
	void updateBias();
	bool threadInit();
	void threadRelease();
	void run();
	void setPositionControl();
	void setForceControl();
	void setFreeMotionControl(bool flag);

private:
	void Configure();
	void PositionControl();
	void ForceControl();
	void FreeMotionControl();
	void (OmegaATIThread::*AdjuctControlOutput)(void); //ForceControl, PositionControl or FreeMotionControl
	bool InitOmegaCommon();
	bool OmegaSetForceControl();
	bool OmegaSetPositionControl();

private:
	bool _positionControl;
	OmegaData _omegaData;
	ResourceFinder& _rsf;

	ForceTorqueData _forceTorqueData;
	PidController _zpController;


	OmegaForceController _xForceController;
	OmegaForceController _yForceController;
	OmegaForceController _zForceController;


	OmegaFTHybridController _zOmegaFTController;

	// Decide during runtime which subcontroller to choose
	PidController* _xController;
	PidController* _yController;
	PidController* _zController;

	OmegaATIPubThread* _omegaATIPubThread;

	bool _ftNotBiased;
	double _stepSize;
	std::clock_t _time;

	double _ftZForce;
	pidParams_t _pidPosCtrl_filterOn; 
	pidParams_t _pidPosCtrl_filterOff;
	pidParams_t _pidParams_omegaForceCtrl;
	pidParams_t _pidParams_FTForceCtrl;
};

