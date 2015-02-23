/*
* Author: Nawid Jamali
* Project: TACMAN
*/

//#include "dhdc.h" // Omega haptic device
#include "drdc.h"
#include "omegaATIThread.h"
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/sig/Vector.h>
#include <string>
#include <iostream>
#include <vector>
#include "pidController.h"
#include <ctime>
using namespace std;
using namespace yarp::os;


#define FT_Z 2

void OmegaATIThread::PositionControl()
{


	// Read the force/torque data
	double ftFx, ftFy, ftFz;
	_forceTorqueData.getBiasedForces(&ftFx, &ftFy, &ftFz);
	//printf("ATI Fx: % 3.4f, Fy: % 3.4f, Fz: % 3.4f\n", ftFx, ftFy, ftFz);

	// Read Omega's Position
	double x, y, z;
	_omegaData.getAxesPos(&x, &y, &z);

	// Get controller offset
	double zPos = _zpController.update(ftFz);

	// Set the tracking position setpoint
	drdTrackPos(x, y, z + zPos);

	// Update omega position
	_omegaData.setZ(z + zPos);

}

void OmegaATIThread::ForceControl()
{
	// Read ATI force/torque data
	double ftFx, ftFy, ftFz;
	_forceTorqueData.getBiasedForces(&ftFx, &ftFy, &ftFz);
	//printf("ATI Fx: % 3.4f, Fy: % 3.4f, Fz: % 3.4f\n", ftFx, ftFy, ftFz);

	// Read omega force data
	double omegaFx, omegaFy, omegaFz;
	dhdGetForce(&omegaFx, &omegaFy, &omegaFz);
	//printf("Omega F: % 3.4f, % 3.4f, % 3.4f\n", omegaFx, omegaFy, omegaFz);
	
	// Read omega position data
	double ox, oy, oz;
	dhdGetPosition(&ox, &oy, &oz);
	//printf("Omega P: % 3.4f, % 3.4f, % 3.4f\n", ox, oy, oz);

	
	
	double velPos[DHD_MAX_DOF];
	drdGetVelocity(velPos);

	//double fx = _xForceController.update(ox, velPos[0]);
	//double fy = _yForceController.update(oy, velPos[1]);
	//double fz = _zForceController.update(oz, velPos[2]);
	
	//double fz = _zOmegaFTController.getSetpoint() + _zOmegaFTController.update(ftFz);
	//double fz = _zOmegaFTController.update(oz, velPos[3], ftFz);
	
	double fx = _xController->update(ox, velPos[0], ftFx);
	double fy = _yController->update(oy, velPos[1], ftFy);
	double fz = _zController->update(oz, velPos[2], ftFz);

	double f[8];
	memset(f, 0, sizeof(f));
	f[0] = fx;
	f[1] = fy;
	f[2] = fz;

	drdSetForceAndTorqueAndGripperForce(f);

}

void OmegaATIThread::run()
{

	//printf("Time: % 3.0f\n",  (std::clock() - _time) / (double)(CLOCKS_PER_SEC / 1000));
	//_time = std::clock();

	
	_forceTorqueData.updateData();
	if(_ftNotBiased)
	{
		updateBias();
		_ftNotBiased = false;
	}
	_omegaATIPubThread->publishData();

#ifdef USE_POSITION_CONTROLLER
	this->PositionControl();
#endif

#ifdef USE_FORCE_CONTROLLER
	this->ForceControl();
#endif




}

bool OmegaATIThread::threadInit()
{

	bool ret = true;
	_ftNotBiased = true;
	_stepSize = 0.0002;



	// Initialise the omega device
#ifdef USE_POSITION_CONTROLLER
	ret = init_omega_drd();
#endif

#ifdef USE_FORCE_CONTROLLER
	ret = init_omega_dhd();
#endif


	_omegaATIPubThread = new OmegaATIPubThread(1, &_forceTorqueData, &_omegaData);
	_omegaATIPubThread->start();


	// PID 
	_zpController.setKp(OMEGA_Z_KP);
	_zpController.setKi(OMEGA_Z_KI);
	_zpController.setKd(OMEGA_Z_KD);
	_zpController.setOutMax(OMEGA_Z_MAX);
	_zpController.setOutMin(OMEGA_Z_MIN);
	_zpController.setSetpoint(OMEGA_Z_SETPOINT);

	
	_xForceController.setKp(FORCE_KP);
	_xForceController.setKd(FORCE_KD);
	_xForceController.setKi(FORCE_KI);
	_xForceController.setOutMax(OMEGA_FORCE_MAX);
	_xForceController.setOutMin(-OMEGA_FORCE_MAX);
	_xForceController.setSetpoint(0);

	_yForceController.setKp(FORCE_KP);
	_yForceController.setKd(FORCE_KD);
	_yForceController.setKi(FORCE_KI);
	_yForceController.setOutMax(OMEGA_FORCE_MAX);
	_yForceController.setOutMin(-OMEGA_FORCE_MAX);
	_yForceController.setSetpoint(0);

	_zForceController.setKp(FORCE_KP);
	_zForceController.setKd(FORCE_KD);
	_zForceController.setKi(FORCE_KI);
	_zForceController.setOutMax(OMEGA_FORCE_MAX);
	_zForceController.setOutMin(-OMEGA_FORCE_MAX);
	_zForceController.setSetpoint(0);


	_zOmegaFTController.setKp(FT_FORCE_KP);
	_zOmegaFTController.setKd(FT_FORCE_KD);
	_zOmegaFTController.setKi(FT_FORCE_KI);
	_zOmegaFTController.setOutMax(FT_FORCE_MAX);
	_zOmegaFTController.setOutMin(-FT_FORCE_MAX);
	_zOmegaFTController.setSetpoint(FT_SETPOINT);

	// Tentatitive, to be changed during runtime
	_xController = &_xForceController;  // Force controller
	_yController = &_yForceController;  // Force controller
	_zController = &_zOmegaFTController; //Hybrid controller

	
	this->UpdateOmegaPosition();

	cout < "\nOmega initialised\n";
	return ret;
}

void OmegaATIThread::threadRelease()
{


	// Close the Omega haptic device
#ifdef USE_POSITION_CONTROLLER
	drdStop();
	drdClose();
#endif // USE_POSITION_CONTROLLER

#ifdef USE_FORCE_CONTROLLER
	dhdEnableForce(DHD_OFF);
	dhdSleep(1);
	dhdStop();
	dhdClose();
#endif // USE_FORCE_CONTROLLER

	_omegaATIPubThread->stop();
	_omegaATIPubThread->threadRelease();
	delete _omegaATIPubThread;
}


bool init_omega()
{
		// Configuration of the Omega device
	dhdEnableExpertMode ();

	// open the first available device
	if(drdOpen () < 0)
	{
		printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
		dhdSleep (2.0);
		return false;
	}

	if (!drdIsSupported()) 
	{
		printf ("unsupported device\n");
		printf ("exiting...\n");
		dhdSleep (2.0);
		drdClose ();
		return false;
	}

	// identify device
	printf ("%s device detected\n\n", dhdGetSystemName());

	// start robot control loop
	if (drdStart() < 0) 
	{
		printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
		dhdSleep (2.0);
		return false;
	}

	drdSetEncPGain(8); //TDODO: config file parameter
	drdSetEncIGain(16);

}

bool init_omega_dhd()
{

	init_omega();

	drdRegulatePos(false);
	drdRegulateGrip(false);
	drdRegulateRot(false);
	drdEnableFilter(false);
	
	return true;
	
}

// Initialise the omega device
bool init_omega_drd()
{
	init_omega();

	double px, py, pz;
	dhdGetPosition(&px, &py, &pz);
	
	if (drdMoveToPos (px, py, pz) < 0) 
	{
		printf ("error: failed to move to central position (%s)\n", dhdErrorGetLastStr ());
		dhdSleep (2.0);
		return false;
	}


	drdEnableFilter(false);

	return true;
}

void OmegaATIThread::UpdateOmegaPosition()
{
	_omegaData.updateData();

	double px, py, pz;
	_omegaData.getAxesPos(&px, &py, &pz);
	_xForceController.setSetpoint(px);
	_yForceController.setSetpoint(py);
	_zForceController.setSetpoint(pz);

}

void OmegaATIThread::stepUp()
{
	_omegaData.setZ(_omegaData.getZ() + _stepSize);
}

void OmegaATIThread::stepDown()
{
	_omegaData.setZ(_omegaData.getZ() - _stepSize);
}

void OmegaATIThread::updateBias()
{
	_omegaData.setBias();
	_forceTorqueData.setBias();
	_ftNotBiased = false;


}

void OmegaATIThread::stepDownTest()
{
	_omegaData.setZ(_omegaData.getZ() - (_stepSize * 10));
}