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
	double zPos = _zController.update(ftFz);

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
	printf("ATI Fx: % 3.4f, Fy: % 3.4f, Fz: % 3.4f\n", ftFx, ftFy, ftFz);

	// Read omega force data
	double omegaFx, omegaFy, omegaFz;
	dhdGetForce(&omegaFx, &omegaFy, &omegaFz);
	//printf("Omega F: % 3.4f, % 3.4f, % 3.4f\n", omegaFx, omegaFy, omegaFz);
	
	// Read omega position data
	double ox, oy, oz;
	dhdGetPosition(&ox, &oy, &oz);
	//printf("Omega P: % 3.4f, % 3.4f, % 3.4f\n", ox, oy, oz);

	//double vx, vy, vz;
	//dhdGetLinearVelocity(&vx, &vy, &vz);
	
	double velPos[DHD_MAX_DOF];
	drdGetVelocity(velPos);

	//double omdPx, omdPy, omdPz; 
	//_omegaData.getAxesPos(&omdPx, &omdPy, &omdPz);

	

	double fx = _xForceController.update(ox, velPos[0]);
	double fy = _yForceController.update(oy, velPos[1]);
	//double fz = _zForceController.update(oz, velPos[2]);
	
	double fz = _zOmegaFTController.getSetpoint() + _zOmegaFTController.update(ftFz);

	double omFx, omFy, omFz;
	_omegaForce.getFroces(&omFx, &omFy, &omFz);

	double f[8];
	memset(f, 0, sizeof(f));
	f[0] = fx;
	f[1] = fy;
	f[2] = fz;

	drdSetForceAndTorqueAndGripperForce(f);
	
	
	
	_omegaForce.setForces(
				omFx,
		        omFy,
				omFz);
				
	//double fZ = _xForceController(ftFz
	//double vx, vy, vz;
	//dhdGetLinearVelocity(&vx, &vy, &vz);

		//dhdSetForce(fx, fy,fz);



}

void OmegaATIThread::run()
{

	printf("Time: % 3.0f\n",  (std::clock() - _time) / (double)(CLOCKS_PER_SEC / 1000));
	_time = std::clock();

	// Read the data, all data should be read here
	Bottle *ft_input = _port_ft.read(); //  Read f/t data 

	_timeStamp.update(); // Update the time stamp for the ports

	_forceTorqueData.updateData(ft_input);

	if(_ftNotBiased)
	{
		updateBias();
		_ftNotBiased = false;
	}

#ifdef USE_POSITION_CONTROLLER
	this->PositionControl();
#endif

#ifdef USE_FORCE_CONTROLLER
	this->ForceControl();
#endif


	
	
	//double fx = -400 * px - 20 * vx; //_xController.update(px, vz);  // -400 * px - 20 * vx;
	//double fy = -400 * py - 20 * vy;
	//double fz =-4;// -100 * pz;

//	double ftFx, ftFy, ftFz;
//	_forceTorqueData.getBiasedForces(&ftFx, &ftFy, &ftFz);
//	printf("ATI Fx: % 3.4f, Fy: % 3.4f, Fz: % 3.4f\n", ftFx, ftFy, ftFz);


	//dhdSetForce(fx, fy,fz);

	//drdMoveToPos(x,y,zPos);

	//drdGetEncMoveParam(&x,&y, &z);

	//printf("A: %f, V: %f, J: %f\n", x,y,z);
	

	//////////////////////////
    // Move to another thread
	//////////////////////////



	/*
	Bottle& ft_output = _port_ft.prepare();
	ft_output.clear();
	ft_output.addDouble(ftFx);
	ft_output.addDouble(ftFy);
	ft_output.addDouble(ftFz);
	_port_ft.setEnvelope(_timeStamp);
	_port_ft.write();

	double ftFxFilt, ftFyFilt, ftFzFilt;
	_forceTorqueData.getFilteredForces(&ftFxFilt, &ftFyFilt, &ftFzFilt);
	//printf("ATI Fx: % 3.4f, Fy: % 3.4f, Fz: % 3.4f\n", ftFxFilt, ftFyFilt, ftFzFilt);
	Bottle& ftFiltered_output = _port_ftFiltered.prepare();
	ftFiltered_output.clear();
	ftFiltered_output.addDouble(ftFxFilt);
	ftFiltered_output.addDouble(ftFyFilt);
	ftFiltered_output.addDouble(ftFzFilt);
	_port_ftFiltered.setEnvelope(_timeStamp);
	_port_ftFiltered.write();

	*/


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

	_port_ft.open("/OmegaATI/ft");
	_port_ftFiltered.open("/OmegaATI/ftFiltered");
	_port_fingertip.open("/OmegaATI/fingertip");
	_port_omega.open("/OmegaATI/omegaPosition");

	Network::connect("/SkinTableTop/skin/fingertip","/OmegaATI/fingertip");
	Network::connect("/NIDAQmxReader/data/real:o","/OmegaATI/ft");

	// PID 
	_zController.setKp(OMEGA_Z_KP);
	_zController.setKi(OMEGA_Z_KI);
	_zController.setKd(OMEGA_Z_KD);
	_zController.setOutMax(OMEGA_Z_MAX);
	_zController.setOutMin(OMEGA_Z_MIN);
	_zController.setSetpoint(OMEGA_Z_SETPOINT);

	
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
}


bool init_omega_dhd()
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

	
	// goto workspace
	//double position[DHD_MAX_DOF], rot[3][3];
	
	//drdGetPositionAndOrientation(position, rot);

	//if (drdMoveToPos(position[0], position[1], position[2]) < 0) 
	//{
		//printf ("error: failed to move to central position (%s)\n", dhdErrorGetLastStr ());
		//dhdSleep (2.0);
		//return false;
	//}

	//while(drdIsMoving())
	//	__nop;
	

	drdRegulatePos(false);
	drdRegulateGrip(false);
	drdRegulateRot(false);

	drdEnableFilter(false);


	/*
	// Configuration of the Omega device
	dhdEnableExpertMode ();

	
	// open the first available device
	if(dhdOpen () < 0)
	{
		printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
		dhdSleep (2.0);
		return false;
	}

	// identify device
	printf ("%s device detected\n\n", dhdGetSystemName());

	dhdEnableForce(DHD_ON);

	dhdSleep(1); */
	
	return true;
	
}

// Initialise the omega device
bool init_omega_drd()
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

	// initialize if necessary
	if (!drdIsInitialized() && (drdAutoInit() < 0)) 
	{
		printf ("error: initialization failed (%s)\n", dhdErrorGetLastStr ());
		dhdSleep (2.0);
		return false;
	}

	// start robot control loop
	if (drdStart() < 0) 
	{
		printf ("error: control loop failed to start properly (%s)\n", dhdErrorGetLastStr ());
		dhdSleep (2.0);
		return false;
	}

	drdSetEncPGain(8); //TDODO: config file parameter
	drdSetEncIGain(16);

	double px, py, pz;
	dhdGetPosition(&px, &py, &pz);
	
	if (drdMoveToPos (px, py, pz) < 0) 
	{
		printf ("error: failed to move to central position (%s)\n", dhdErrorGetLastStr ());
		dhdSleep (2.0);
		return false;
	}

	//double amax, vmax, jerk;
	//drdGetPosTrackParam(&amax, &vmax, &jerk);
	//printf("Amax: % 2.3f, Vmax: % 2.3f, Jerk: % 2.3f\n", amax, vmax, jerk);
	//drdSetPosTrackParam(amax, vmax/1000, jerk/1000);
	//drdGetPosTrackParam(&amax, &vmax, &jerk);
	//printf("Amax: % 2.3f, Vmax: % 2.3f, Jerk: % 2.3f\n", amax, vmax, jerk);

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