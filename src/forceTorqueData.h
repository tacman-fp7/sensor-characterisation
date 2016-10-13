#pragma once

#include <iostream>
#include <array>
#include <queue>
#include <yarp/os/Mutex.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>


#define FILTER_WINDOW 10
#define FT_CHANNELS 6

using namespace yarp::os;
using namespace std;

class ForceTorqueData
{
public:
	ForceTorqueData();
	bool updateData();
	void setForces(double fx, double fy, double fz);
	void setTorques(double tx, double ty, double tz);
	void setBias();
	void setBias(double fxBias, double fyBias, double fzBias);
	void getForces(double *fx, double *fy, double *fz);
	void publishData(yarp::os::Stamp& timeStamp);
	
	void getBiasedForces(double *fx, double *fy, double *fz)
	{
		_mutex.lock();
		*fx = _fx - _fxBias; *fy = _fy - _fyBias; *fz = _fz - _fzBias;
		_mutex.unlock();
	}
	void getFilteredForces(double *fx, double *fy, double *fz)
	{
		_mutex.lock();
		*fx = _fxFiltered; *fy = _fyFiltered; *fz = _fzFiltered;
		_mutex.unlock();
	}
private:

	void resetFilterBuffer()
	{
		_mutex.lock();
		_filterBuffer.empty();
		array<double, FT_CHANNELS> tempArray;
		tempArray.fill(0);
		for (int i = 0; i < FILTER_WINDOW; i++)
			_filterBuffer.push(tempArray);
		_mutex.unlock();

	}
	double _fx, _fy, _fz, _tx, _ty, _tz, _resultant;
	double _fxBias, _fyBias, _fzBias, _txBias, _tyBias, _tzBias;
	double _fxFiltered, _fyFiltered, _fzFiltered, _txFiltered, _tyFiltered, _tzFiltered;
	queue< array<double, FT_CHANNELS> > _filterBuffer;
	Mutex _mutex;
	yarp::os::Stamp _timeStamp;

	BufferedPort<Bottle> _port_ft;
	BufferedPort<Bottle> _port_ftFiltered;

};