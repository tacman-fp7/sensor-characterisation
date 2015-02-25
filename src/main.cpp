/*
* Author: Nawid Jamali
* Project: TACMAN
*/
#include "omegaATIThread.h"
#include <iostream>
#include "dhdc.h" // Omega haptic device
#include "drdc.h"
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/all.h> //

using namespace std;
using namespace yarp::os;

int main(int argc, char *argv[]) {

	Network yarp;
	bool freeCtrl = false;
	if (!yarp.checkNetwork())
	{
		printf("No yarp network, quitting\n");
		return false;
	}

	ResourceFinder rsf;
	rsf.setVerbose();
	rsf.setDefaultConfigFile("omegaATIConf.ini");
	rsf.configure(argc, argv);

	int  period = rsf.check("period", 1).asInt();

	
	OmegaATIThread experimentThread(period, rsf);
	experimentThread.start();
	dhdSleep(2);

	while(true)
	{
		if(dhdKbHit())
		{
			char key = dhdKbGet ();
			if(key == 'q' || key == 'Q') // quit the experiment
				break;
			else if(key == 'r' || key == 'R') // update the position of the Omega device.
			{
				// Allow free movement of the omega device
				freeCtrl = !freeCtrl;
				experimentThread.setFreeMotionControl(freeCtrl);
			}
			else if(key == 'p' || key == 'P') // update the position of the Omega device.
			{
				if(freeCtrl){
					freeCtrl = false;
					experimentThread.setFreeMotionControl(freeCtrl);
				}
				experimentThread.setPositionControl();
			}
			else if(key == 'f' || key == 'f') // update the position of the Omega device.
			{
				if(freeCtrl){
					freeCtrl = false;
					experimentThread.setFreeMotionControl(freeCtrl);
				}
				experimentThread.setForceControl();
			}
			else if(key == 'E' || key == 'E') 
			{
				// Read the config file for experiment details
			}
			else if( key == 'U')
			{
				experimentThread.stepUp();
			}
			else if( key == 'D')
			{
				experimentThread.stepDown();
			}
			else if ( key == 'b' || key == 'B')
				experimentThread.updateBias();
			else if ( key == 's')
				experimentThread.stepDownTest();

		}
	}

	// Stop the contorller
	experimentThread.stop();

	return 0;
}

