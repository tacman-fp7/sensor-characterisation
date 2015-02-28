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
#include "pidPositionController.h"
using namespace std;
using namespace yarp::os;


void OmegaATIThread::setPositionControl()
{
	_positionControl = true;
	this->OmegaSetPositionControl();
	AdjuctControlOutput = &OmegaATIThread::PositionControl;
}

void OmegaATIThread::setForceControl()
{
	_positionControl = false;
	this->OmegaSetForceControl();
	AdjuctControlOutput = &OmegaATIThread::ForceControl;
}

void OmegaATIThread::setFreeMotionControl(bool on)
{
	if(on)
	{
		if(_positionControl)
		{

			drdRegulatePos(false);
			double f[8];
			memset(f, 0, sizeof(f));
			drdSetForceAndTorqueAndGripperForce(f);
		}
		else
		{
			double f[8];
			memset(f, 0, sizeof(f));
			f[2] = 0;
			drdSetForceAndTorqueAndGripperForce(f);
		}
		// Set the controler to free motion control
		AdjuctControlOutput = &OmegaATIThread::FreeMotionControl;
	}
	else
	{
		this->UpdateOmegaPosition();

		if(_positionControl)
		{
			drdRegulatePos(true);
			AdjuctControlOutput = &OmegaATIThread::PositionControl;
		}
		else
		{
			AdjuctControlOutput = &OmegaATIThread::ForceControl;
		}
	}
}

void OmegaATIThread::FreeMotionControl()
{
	// Do nothing
}
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

	double xPos = _xControllerPos->update(ftFx);//_xPositionController.update(ftFx);
	double yPos = _yControllerPos->update(ftFy); //_yPositionController.update(ftFy);
	double zPos = _zControllerPos->update(ftFz);//_zPositionController.update(ftFz);

	//printf("% 3.3f\t", xPos);
	// Set the tracking position setpoint
	drdTrackPos(x + xPos, y + yPos, z + zPos);

	// Update omega position
	//_omegaData.setZ(z + zPos);
	_omegaData.setAxesPos(x + xPos, y + yPos, z + zPos);

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

	// The control step can be defined at runtime by user. 
	// The system starts with force controller.
	(this->*AdjuctControlOutput)();

}

bool OmegaATIThread::threadInit()
{

	bool ret = true;
	_ftNotBiased = true;
	_stepSize = 0.0002; //TODO: move to the config file

	// ReadConfiguration file
	this->Configure();


	// Initialise omega device
	ret = this->InitOmegaCommon();

	// We start in force control mode, which is smoother
	AdjuctControlOutput = &OmegaATIThread::ForceControl;

	ret = this->OmegaSetForceControl();

	// Set the current position of the device as the desired position
	this->UpdateOmegaPosition();

	// Start the publishing thread
	_omegaATIPubThread = new OmegaATIPubThread(1, &_forceTorqueData, &_omegaData);
	_omegaATIPubThread->start();


	// PID PositionController
	_pidPosCtrl_filterOff.outMax = _omegaData.getZLimitMax();
	_pidPosCtrl_filterOff.outMin = _omegaData.getZLimitMin();

	_zPositionController.InitController(_pidPosCtrl_filterOff);
	_zPositionController.setSetpoint(_ftZForce);
	_xPositionController.InitController(_pidPosCtrl_filterOff);
	_xPositionController.setSetpoint(0); //TODO: check
	_yPositionController.InitController(_pidPosCtrl_filterOff);
	_yPositionController.setSetpoint(0); //TODO: check

	// PID Omega Froce controller, maintains position
	_xForceController.InitController(_pidParams_omegaForceCtrl);
	_yForceController.InitController(_pidParams_omegaForceCtrl);
	_zForceController.InitController(_pidParams_omegaForceCtrl);

	// PID Uses Force/Torque to change forces
	_zOmegaFTController.InitController(_pidParams_FTForceCtrl);
	_zOmegaFTController.setSetpoint(_ftZForce);
	_xOmegaFTController.InitController(_pidParams_FTForceCtrl_x);
	_xOmegaFTController.setSetpoint(0);
	_yOmegaFTController.InitController(_pidParams_FTForceCtrl_y);
	_yOmegaFTController.setSetpoint(0);

	// Tentatitive, to be changed during runtime
	_xController = &_xForceController;  // Force controller
	_yController = &_yForceController;  // Force controller
	_zController = &_zForceController; //&_zOmegaFTController; //Hybrid controller

	// Urgh
	_xControllerPos = &_xZeroPositionController;
	_yControllerPos = &_yZeroPositionController;
	_zControllerPos = &_zPositionController;

	cout < "\nOmega initialised\n";
	return ret;
}



void OmegaATIThread::threadRelease()
{


	// Close the Omega haptic device


	dhdEnableForce(DHD_OFF);
	dhdSleep(0.1); // Wait for a short while before closing the system

	drdStop();
	drdClose();



	_omegaATIPubThread->stop();
	delete _omegaATIPubThread;
}

bool OmegaATIThread::InitOmegaCommon()
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

	dhdSetEffectorMass(0.215); //Value determined using trial and error
	drdEnableFilter(true);
	//Initial position
	drdMoveToPos(0, 0, 0); // Safe position 
	drdEnableFilter(false);

}

bool OmegaATIThread::OmegaSetForceControl()
{

	drdRegulatePos(false);
	drdRegulateGrip(false);
	drdRegulateRot(false);
	drdEnableFilter(false);

	//double f[8];
	//memset(f, 0, sizeof(f));
	//f[2] = -0.5;
	//drdSetForceAndTorqueAndGripperForce(f);

	return true;

}

bool OmegaATIThread::OmegaSetPositionControl()
{


	//printf("position control requested\n");

	_omegaData.updateData();
	double px, py, pz;
	_omegaData.getAxesPos(&px, &py, &pz);
	//printf("position1\n");
	drdRegulatePos(true);
	//printf("position2\n");
	drdEnableFilter(true);
	if (drdMoveToPos (px, py, pz, false) < 0) 
	{
		printf ("error: failed to move to central position (%s)\n", dhdErrorGetLastStr ());
		dhdSleep (2.0);
		return false;
	}
	//printf("position3\n");
	dhdSleep(1);
	//printf("position4\n");
	drdEnableFilter(false);

	//printf("position enabled\n");

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

void OmegaATIThread::runExperiment(ResourceFinder& rf)
{

	//ResourceFinder& expeRSF = rf.findNestedResourceFinder( "Experiment");

	Bottle& experiments = rf.findGroup("Experiment");
	if(experiments.isNull())
	{
		printf("No experiments in the file\n");
		return;
	}

	int nExp = experiments.size();
	printf("Experiment steps: %d\n", nExp);

	for(int i = 1; i < nExp; i++){
		printf("%s\n", experiments.get(i).toString());
		Bottle& step = rf.findGroup(experiments.get(i).asString());
		ReadExperimentDetails(step); // updates the _experimentDetails

		// Run the step
		printf("Running the step...");
		if(_experimentData.isConsecutiveForce)
		{
			printf("\nconsecutive\n");
			performExperimentConsecStep();
		}
		else
		{




			// Should check step size
			if(_experimentData.stepSize == 0)
			{
				performExperimentStep();
			}
			else
			{

				double vx = _experimentData.sampleLocationEndpoint.at(0) - _experimentData.sampleLocation.at(0);
				double vy = _experimentData.sampleLocationEndpoint.at(1) - _experimentData.sampleLocation.at(1);	
				double mag = sqrt(vx * vx + vy * vy);
				// Unit vector
				vx /= mag;
				vy /= mag;

				int nSteps = int(mag / (_experimentData.stepSize / 1000));

				printf("Expected steps for mag (%f): %d \n", mag, nSteps);
				// Go through steps


				for (int step = 0; step < nSteps  ; step++)
				{
					printf("Step %02d\n", step); 

					_experimentData.sampleLocation.at(0) += vx* step * _experimentData.stepSize / 1000;
					_experimentData.sampleLocation.at(1) += vy* step * _experimentData.stepSize / 1000;

					performExperimentStep();
					dhdSleep(_experimentData.hysteresisDelay);


				}
			}
		}
		printf("Done.\n\n");
	}

	// Experiment is done, reduce force TODO: rename variable for defaultFzFroce
	_zOmegaFTController.setSetpoint(_ftZForce);
	_zPositionController.setSetpoint(_ftZForce); 
}

void OmegaATIThread::performExperimentConsecStep()
{
	// I assume the position is same as the current position
	UpdateOmegaPosition(); //update the position of the omega device in memory

	if(_experimentData.controlStrategy == 1) // 1 is for forceController
	{
		if(_experimentData.forceAxis != 2)
		{
			// In this case I have to change the PID controller
			_xOmegaFTController.setRampSetpoint(_experimentData.forceSetpoint.at(0));
			_yOmegaFTController.setRampSetpoint(_experimentData.forceSetpoint.at(1));
			_xController = &_xOmegaFTController;
			_yController = &_yOmegaFTController;

		}
		//_zOmegaFTController.setSetpoint(_experimentData.forceSetpoint.at(2));
		_zOmegaFTController.setRampSetpoint(_experimentData.forceSetpoint.at(2));
		setForceControl();
	}
	else if(_experimentData.controlStrategy == 2) // 2 is for positon controller
	{

		if(_experimentData.forceAxis != 2)
		{
			_xPositionController.setSetpoint(_experimentData.forceSetpoint.at(0));
			_yPositionController.setSetpoint(_experimentData.forceSetpoint.at(1));
			//_xControllerPos = &_xPositionController;
			//_yControllerPos = &_yPositionController;
		}
		_zPositionController.setSetpoint(_experimentData.forceSetpoint.at(2));

		setPositionControl();
	}

	dhdSleep(_experimentData.contactPeriod);
}

void OmegaATIThread::performExperimentStep()
{



	//Change to free motion controller
	setFreeMotionControl(true);
	drdRegulatePos(true);
	drdEnableFilter(true);
	// Move to 0 posisition to avoid any collisions
	drdMoveToPos(_experimentData.sampleLocation.at(0),_experimentData.sampleLocation.at(1),0);

	/*********************/
	/// Use the motion control to get zero, then use force controll to change the position
	// Then use the hybrid control


	// Move to sample point
	drdMoveToPos(_experimentData.sampleLocation.at(0),
		_experimentData.sampleLocation.at(1),
		_experimentData.sampleLocation.at(2));


	// Keep going down until force is greater than 0.1 N;

	double fx, fy, fz;
	double offset = 0;
	_forceTorqueData.getBiasedForces(&fx, &fy, &fz);
	while(fz > -0.02){
		if(_experimentData.forceSetpoint.at(2) == 0)
			break;
		offset -= 0.0001;
		drdMoveToPos(_experimentData.sampleLocation.at(0),
			_experimentData.sampleLocation.at(1),
			_experimentData.sampleLocation.at(2)+offset);

		//printf("Not making contact yet\r");
		_forceTorqueData.getBiasedForces(&fx, &fy, &fz);
	}


	//dhdSleep(1);

	drdEnableFilter(false);
	// Change the x, y force controller to omega force to make sure if there was a
	// consecutive force sequce applied, we undo its effects
	_xController = &_xForceController;
	_yController = &_yForceController;
	_zController = &_zForceController; //_zController = &_zOmegaFTController;

	UpdateOmegaPosition(); //update the position of the omega device in memory


	double f[8];
	memset(f, 0, sizeof(f));
	//f[2] = -0.5; // makes sure contact is maintained
	drdSetForceAndTorqueAndGripperForce(f);




	// This part is to avoid jumping when we deregulare position, clean later
	double px, py, pz;
	_omegaData.getAxesPos(&px, &py, &pz);
	// set to my controller
	_xForceController.setSetpoint(px);
	_yForceController.setSetpoint(py);
	_zForceController.setSetpoint(pz);

	_xForceController.setRampSetpoint(_experimentData.sampleLocation.at(0));
	_yForceController.setRampSetpoint(_experimentData.sampleLocation.at(1));
	_zForceController.setRampSetpoint(_experimentData.sampleLocation.at(2));
	// end of set to my controller

	_zOmegaFTController.setSetpoint(-0.1); // maintain contact
	_zController = & _zOmegaFTController;



	drdRegulatePos(false);


	setForceControl();

	dhdSleep(0.3);



	if(_experimentData.controlStrategy == 1) // 1 is for forceController
	{
		if(_experimentData.forceAxis == 2){

			_zOmegaFTController.setRampSetpoint(_experimentData.forceSetpoint.at(2));
			_zController = &_zOmegaFTController;
		}
		setForceControl();
		dhdSleep(0.2);

		// Experimental
		_xOmegaFTController.setSetpoint(_experimentData.forceSetpoint.at(0));
		//_xOmegaFTController.setRampSetpoint(_experimentData.forceSetpoint.at(0));
		_yOmegaFTController.setSetpoint(_experimentData.forceSetpoint.at(1));
		//_yOmegaFTController.setRampSetpoint(_experimentData.forceSetpoint.at(1));
		_xController = &_xOmegaFTController;
		_yController = &_yOmegaFTController;
		// End experimental 
	}
	else if(_experimentData.controlStrategy == 2)
	{
		if(_experimentData.forceAxis == 2)
			_zPositionController.setSetpoint(_experimentData.forceSetpoint.at(2));

		setPositionControl();
	}

	dhdSleep(_experimentData.contactPeriod);


}


void OmegaATIThread::ReadExperimentDetails(Bottle& bottle)
{
	_experimentData.controlStrategy = bottle.check("controlStrategy", Value(0)).asInt();
	_experimentData.contactPeriod = bottle.check("contactPeriod", Value(0.0)).asDouble();
	_experimentData.hysteresisDelay = bottle.check("hysteresisDelay", Value(0.0)).asDouble();
	_experimentData.stepSize = bottle.check("stepSize", Value(0)).asDouble();
	_experimentData.forceAxis = bottle.check("forceAxis", Value(-1)).asInt();
	_experimentData.isConsecutiveForce = bottle.check("isConsecutiveForce", Value(0)).asInt();
	Bottle* forceList = bottle.find("forceSetpoint").asList();
	Bottle* sampleLocationList = bottle.find("sampleLocation").asList();
	Bottle* sampleLocationEndpointList =  bottle.find("sampleLocationEndpoint").asList();

	if((forceList->isNull() ||  sampleLocationEndpointList->isNull()))
	{
		printf("Warning: no force or sample location endpoint could be read. Skipping the step\n");
		return;
	}

	if((forceList->isNull() ||  sampleLocationList->isNull()))
	{
		printf("Warning: no force or sample location could be read. Skipping the step\n");
		return;
	}

	if( (forceList->size() != 3) || (sampleLocationList->size() != 3))
	{
		printf("Warning: foce or sample location is not a tripple. Skipping the step\n");
		return;
	}

	_experimentData.forceSetpoint.clear();
	_experimentData.sampleLocation.clear();
	_experimentData.sampleLocationEndpoint.clear();

	for(int i = 0; i < 3; i++)
	{
		_experimentData.forceSetpoint.push_back(forceList->get(i).asDouble());
		_experimentData.sampleLocation.push_back(sampleLocationList->get(i).asDouble());
		_experimentData.sampleLocationEndpoint.push_back(sampleLocationEndpointList->get(i).asDouble());
	}

	printf("Control strategy: %d\n", _experimentData.controlStrategy);
	printf("Foce setpoint: (% 3.3f, % 3.3f, % 3.3f)\n", _experimentData.forceSetpoint.at(0),
		_experimentData.forceSetpoint.at(1), _experimentData.forceSetpoint.at(2));
	printf("Sample location: (% 3.3f, % 3.3f, % 3.3f)\n", _experimentData.sampleLocation.at(0),
		_experimentData.sampleLocation.at(1), _experimentData.sampleLocation.at(2) );
	printf("Sample end point: (% 3.3f, % 3.3f, % 3.3f)\n", _experimentData.sampleLocationEndpoint.at(0),
		_experimentData.sampleLocationEndpoint.at(1), _experimentData.sampleLocationEndpoint.at(2) );
	printf("Step size: % 3.3f\n", _experimentData.stepSize);
	printf("Contact period: % 3.3f\n", _experimentData.contactPeriod);
	printf("Hysteresis delay: % 3.3f\n", _experimentData.hysteresisDelay);
	printf("IsConsecutive: %d\n", _experimentData.isConsecutiveForce);
	printf("Force axis: %d \n\n", _experimentData.forceAxis);


}

void OmegaATIThread::Configure()
{
	// Configure the thread
	cout << "Reading configuration file" << endl;

	Bottle &OmegaConfig = _rsf.findGroup("OmegaConf");
	if(!OmegaConfig.isNull())
	{
		_ftZForce = OmegaConfig.check("DefaultZForce", Value(-0.5)).asDouble();
		Bottle *omegaMinList = OmegaConfig.find("PosLimMin").asList();
		Bottle *omegaMaxList = OmegaConfig.find("PosLimMax").asList();

		if(!(omegaMinList->isNull() || omegaMaxList->isNull()))
		{
			_omegaData.setAxesLimitMin(omegaMinList->get(0).asDouble(),
				omegaMinList->get(1).asDouble(),
				omegaMinList->get(2).asDouble());

			_omegaData.setAxesLimitMax(omegaMaxList->get(0).asDouble(),
				omegaMaxList->get(1).asDouble(),
				omegaMaxList->get(2).asDouble());

			printf("Omega Max: %s\n", omegaMaxList->toString());
			printf("Omega Min: %s\n", omegaMinList->toString());
		}
	}

	Bottle &PIDPosCtrl_FilterOff = _rsf.findGroup("PIDPosCtrl_FilterOff");
	if(!PIDPosCtrl_FilterOff.isNull())
	{

		_pidPosCtrl_filterOff.Kp = PIDPosCtrl_FilterOff.check("Z_KP", Value(0)).asDouble();
		_pidPosCtrl_filterOff.Ki = PIDPosCtrl_FilterOff.check("Z_KI", Value(0)).asDouble();
		_pidPosCtrl_filterOff.Ki /= 100000.0; //Hack
		_pidPosCtrl_filterOff.Kd = PIDPosCtrl_FilterOff.check("Z_KD", Value(0)).asDouble();
		printf("Filter-off: % 1.3e, % 1.3e, % 1.3e\n", _pidPosCtrl_filterOff.Kp, _pidPosCtrl_filterOff.Ki, _pidPosCtrl_filterOff.Kd);
	}

	Bottle &PIDPosCtrl_FilterOn = _rsf.findGroup("PIDPosCtrl_FilterOn");
	if(!PIDPosCtrl_FilterOn.isNull())
	{

		_pidPosCtrl_filterOn.Kp = PIDPosCtrl_FilterOn.check("Z_KP", Value(0)).asDouble();
		_pidPosCtrl_filterOn.Ki = PIDPosCtrl_FilterOn.check("Z_KI", Value(0)).asDouble();
		_pidPosCtrl_filterOn.Ki /= 100000.0; //Hack
		_pidPosCtrl_filterOn.Kd = PIDPosCtrl_FilterOn.check("Z_KD", Value(0)).asDouble();
		printf("Filter-on % 1.3e, % 1.3e, % 1.3e\n", _pidPosCtrl_filterOn.Kp, _pidPosCtrl_filterOn.Ki, _pidPosCtrl_filterOn.Kd);
	}

	Bottle &PIDOmegaForceCtrl = _rsf.findGroup("PIDOmegaForceCtrl");
	if(!PIDOmegaForceCtrl.isNull())
	{
		_pidParams_omegaForceCtrl.Kp = PIDOmegaForceCtrl.check("KP", Value(0)).asDouble();
		_pidParams_omegaForceCtrl.Ki = PIDOmegaForceCtrl.check("KI", Value(0)).asDouble();
		_pidParams_omegaForceCtrl.Kd = PIDOmegaForceCtrl.check("KD", Value(0)).asDouble();
		_pidParams_omegaForceCtrl.outMax = PIDOmegaForceCtrl.check("MAX", Value(0)).asDouble();
		_pidParams_omegaForceCtrl.outMin = -1 * _pidParams_omegaForceCtrl.outMax;
		printf("Omega-force: % 4.2f, % 4.2f, % 4.2f, % 4.2f\n", _pidParams_omegaForceCtrl.Kp, _pidParams_omegaForceCtrl.Ki, _pidParams_omegaForceCtrl.Kd, _pidParams_omegaForceCtrl.outMax );
	}


	Bottle &PIDFTForceCtrl = _rsf.findGroup("PIDFTForceCtrl");
	if(!PIDFTForceCtrl.isNull())
	{
		_pidParams_FTForceCtrl.Kp = PIDFTForceCtrl.check("KP", Value(0)).asDouble();
		_pidParams_FTForceCtrl.Ki = PIDFTForceCtrl.check("KI", Value(0)).asDouble();
		_pidParams_FTForceCtrl.Kd = PIDFTForceCtrl.check("KD", Value(0)).asDouble();
		_pidParams_FTForceCtrl.outMax = PIDFTForceCtrl.check("MAX", Value(0)).asDouble();
		_pidParams_FTForceCtrl.outMin = -1 * _pidParams_FTForceCtrl.outMax;
		printf("Ft-force: % 4.2f, % 4.2f, % 4.2f, % 4.2f\n", _pidParams_FTForceCtrl.Kp, _pidParams_FTForceCtrl.Ki, _pidParams_FTForceCtrl.Kd, _pidParams_FTForceCtrl.outMax);
	}

	Bottle &PIDFTForceCtrl_x = _rsf.findGroup("PIDFTForceCtrl_x");
	if(!PIDFTForceCtrl.isNull())
	{
		_pidParams_FTForceCtrl_x.Kp = PIDFTForceCtrl_x.check("KP", Value(0)).asDouble();
		_pidParams_FTForceCtrl_x.Ki = PIDFTForceCtrl_x.check("KI", Value(0)).asDouble();
		_pidParams_FTForceCtrl_x.Kd = PIDFTForceCtrl_x.check("KD", Value(0)).asDouble();
		_pidParams_FTForceCtrl_x.outMax = PIDFTForceCtrl_x.check("MAX", Value(0)).asDouble();
		_pidParams_FTForceCtrl_x.outMin = -1 * _pidParams_FTForceCtrl.outMax;
		printf("Ft-force-x: % 4.2f, % 4.2f, % 4.2f, % 4.2f\n", _pidParams_FTForceCtrl_x.Kp, _pidParams_FTForceCtrl_x.Ki, _pidParams_FTForceCtrl_x.Kd, _pidParams_FTForceCtrl_x.outMax);
	}

	Bottle &PIDFTForceCtrl_y = _rsf.findGroup("PIDFTForceCtrl_y");
	if(!PIDFTForceCtrl.isNull())
	{
		_pidParams_FTForceCtrl_y.Kp = PIDFTForceCtrl_y.check("KP", Value(0)).asDouble();
		_pidParams_FTForceCtrl_y.Ki = PIDFTForceCtrl_y.check("KI", Value(0)).asDouble();
		_pidParams_FTForceCtrl_y.Kd = PIDFTForceCtrl_y.check("KD", Value(0)).asDouble();
		_pidParams_FTForceCtrl_y.outMax = PIDFTForceCtrl_y.check("MAX", Value(0)).asDouble();
		_pidParams_FTForceCtrl_y.outMin = -1 * _pidParams_FTForceCtrl.outMax;
		printf("Ft-force-y: % 4.2f, % 4.2f, % 4.2f, % 4.2f\n", _pidParams_FTForceCtrl_y.Kp, _pidParams_FTForceCtrl_y.Ki, _pidParams_FTForceCtrl_y.Kd, _pidParams_FTForceCtrl_y.outMax);
	}
}