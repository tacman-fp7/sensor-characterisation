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
	bool posCtrl = false;
	if (!yarp.checkNetwork())
	{
		printf("No yarp network, quitting\n");
		return false;
	}




	OmegaATIThread experimentThread(4);
	experimentThread.start();


	while(true)
	{
		if(dhdKbHit())
		{
			char key = dhdKbGet ();
			if(key == 'q') // quit the experiment
				break;
			else if(key == 'p') // update the position of the Omega device.
			{
				posCtrl = !posCtrl;
				if(posCtrl)
				{
					experimentThread.UpdateOmegaPosition();
					drdStart();
				}
				else
					drdStop();
			}
		}
	}

	experimentThread.stop();


	return 0;
}

