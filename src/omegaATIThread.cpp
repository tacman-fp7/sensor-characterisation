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

using namespace std;
using namespace yarp::os;


#define FT_Z 2

void OmegaATIThread::run()
{

	// Read the data, all data should be read here
	Bottle *ft_input = _port_ft.read(); //  Read f/t data 
	Bottle *fingertip_input = _port_fingertip.read(); // Read fingertip data 

	_timeStamp.update(); // Update the time stamp for the ports

	if(fingertip_input == NULL)
	{
		cout << "Warning! No finger data." << endl;
		return;
	}

	_forceTorqueData.updateData(ft_input);

	double omegaFx, omegaFy, omegaFz;
	dhdGetForce(&omegaFx, &omegaFy, &omegaFz);
	//printf("Omega Forces: %0.4f, %0.4f, %0.4f\n", omegaFx, omegaFy, omegaFz);

	double ftFx, ftFy, ftFz;
	_forceTorqueData.getBiasedForces(&ftFx, &ftFy, &ftFz);
	printf("ATI Fx: % 3.4f, Fy: % 3.4f, Fz: % 3.4f\n", ftFx, ftFy, ftFz);
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

	//printf("PID gains: % 3.4f, % 3.4f, % 3.4f\n", drdGetEncPGain(), drdGetEncIGain(), drdGetEncDGain());
	double x, y, z;
	_omegaData.getAxesPos(&x, &y, &z);
	//cout << "Desired pos: ";
	//printf("% 3.4f, % 3.4f, % 3.4f\n", x, y, z);
	drdMoveToPos(x,y,z);

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



}

bool OmegaATIThread::threadInit()
{

	bool ret = true;
	_stepSize = 0.0002;


	// Initialise the omega device
	ret = init_omega();

	_port_ft.open("/OmegaATI/ft");
	_port_ftFiltered.open("/OmegaATI/ftFiltered");
	_port_fingertip.open("/OmegaATI/fingertip");
	_port_omega.open("/OmegaATI/omegaPosition");

	Network::connect("/SkinTableTop/skin/fingertip","/OmegaATI/fingertip");
	Network::connect("/NIDAQmxReader/data/real:o","/OmegaATI/ft");

	

	

	cout < "\nOmega initialised\n";
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

	drdSetEncPGain(8); //TDODO: config file parameter
	drdSetEncIGain(32);

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
	_omegaData.updateData();
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


}

void OmegaATIThread::stepDownTest()
{
	_omegaData.setZ(_omegaData.getZ() - (_stepSize * 10));
}