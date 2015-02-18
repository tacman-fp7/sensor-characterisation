/*
* Author: Nawid Jamali
* Project: TACMAN
*/
#include "omegaATIThread.h"
#include <iostream>
#include "dhdc.h" // Omega haptic device
#include "drdc.h"
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;





int main(int argc, char *argv[]) {


	Network yarp;
	bool posCtrl = true;
	if (!yarp.checkNetwork())
	{
		printf("No yarp network, quitting\n");
		return false;
	}




	OmegaATIThread experimentThread(1);
	experimentThread.start();
	dhdSleep(2);
	experimentThread.updateBias();
	dhdSleep(1);


	while(true)
	{
		if(dhdKbHit())
		{
			char key = dhdKbGet ();
			if(key == 'q' || key == 'Q') // quit the experiment
				break;
			else if(key == 'p' || key == 'P') // update the position of the Omega device.
			{
				posCtrl = !posCtrl;
				if(posCtrl)
				{
					experimentThread.UpdateOmegaPosition();
					drdStart();
				}
				else
				{
					drdStop();
					dhdSleep(1);
					dhdEnableForce(DHD_ON);
				}
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

	experimentThread.stop();


	return 0;
}

