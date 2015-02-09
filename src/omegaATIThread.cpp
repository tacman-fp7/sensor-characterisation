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


#include <string>
#include <iostream>

using namespace std;
using namespace yarp::os;

#define FT_Z 2
#define OMEGA_UPPER_LIMIT 0.08
#define OMEGA_LOWER_LIMIT -0.065
void OmegaATIThread::run()
{

	// Read the data, all data should be read here
	Bottle *ft_input = _port_ft.read(); //  Read f/t data 
	Bottle *fingertip_input = _port_fingertip.read(); // Read fingertip data 


	if(ft_input == NULL)
	{
		cout << "Warning! No force/torque data." << endl;
		return; 
	}
	else if(fingertip_input == NULL)
	{
		cout << "Warning! No finger data." << endl;
		return;
	}

	// Get the force in the Z direction of the ft sensor
	double ftZ =  ft_input->get(FT_Z).asDouble();

	
	if(!drdIsRunning())
		cout << "Warning! DRD is not running" << endl;

	double targetForce = -12;
	double error = targetForce - ftZ;

	double omegaFx, omegaFy, omegaFz;
	dhdGetForce(&omegaFx, &omegaFy, &omegaFz);
	printf("Omega Forces: %0.4f, %0.4f, %0.4f\n", omegaFx, omegaFy, omegaFz);

   printf("ATI Fz: %0.4f\n", ftZ);
	cout << "Omega Pos: ";
	double x, y, z;
	_omegaData.getAxesPos(&x, &y, &z);
	printf("%0.4f, %0.4f, %0.4f\n", x, y, z);
	double newPosZ = z + 0.1 * error;
	if(newPosZ < OMEGA_LOWER_LIMIT)
		newPosZ = OMEGA_LOWER_LIMIT;
	if(newPosZ > OMEGA_UPPER_LIMIT)
		newPosZ = OMEGA_UPPER_LIMIT;

	double 	drdPos[DHD_MAX_DOF];
	drdGetPositionAndOrientation(drdPos, NULL);
	printf("DRD: %0.4f, %0.4f, %0.4f\n\n", drdPos[0], drdPos[1], drdPos[2]);
	_omegaData.setZ(newPosZ);
	drdMoveToPos(x,y,newPosZ);
}

bool OmegaATIThread::threadInit()
{

	bool ret = true;
	// Initialise the omega device
	ret = init_omega();

	_port_ft.open("/OmegaATI/ft");
	_port_fingertip.open("/OmegaATI/fingertip");

	Network::connect("/SkinTableTop/skin/fingertip","/OmegaATI/fingertip");
	Network::connect("/NIDAQmxReader/data/real:o","/OmegaATI/ft");

	return ret;
}

void OmegaATIThread::threadRelease()
{

	// Close the Omega haptic device
	drdStop();
	drdClose();

}


// Initialise the omega device
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

	// goto workspace center
	if (drdMoveToPos (0.0, 0.0, 0.0) < 0) 
	{
		printf ("error: failed to move to central position (%s)\n", dhdErrorGetLastStr ());
		dhdSleep (2.0);
		return false;
	}
	return true;
}

void OmegaATIThread::UpdateOmegaPosition()
{
	//double px, py, pz, rx, ry, rz;
	//dhdGetPositionAndOrientationRad(&px, &py, &pz, &rx, &ry, &rz);
	//_omegaData.setAxesPos(pz, py, pz);

	double robotPose[DHD_MAX_DOF];
	drdGetPositionAndOrientation( robotPose, NULL); // I am not interested in the orientation matrix
	_omegaData.setAxesPos(robotPose[0], robotPose[1], robotPose[2] );


}