#include "forceTorqueData.h"


forceTorqueData::forceTorqueData()
{

	_fx = _fy = _fz =0;
	_fxBias = _fyBias = _fzBias = 0;
	_fxFiltered = _fyFiltered = _fzFiltered = 0;
	_txFiltered = _tyFiltered = _tzFiltered = 0;

	_timeStamp.update();

	array<double, FT_CHANNELS> tempArray;
	tempArray.fill(0);
	for (int i = 0; i < FILTER_WINDOW; i++)
		_filterBuffer.push(tempArray);
};

bool forceTorqueData::updateData(Bottle *ft_input)
{
	if(ft_input == NULL)
	{
		cout << "Warning! No force/torque data." << endl;
		return false; 
	}


	_mutex.lock();

	_timeStamp.update(); // Get the time stamp associated with the data

	_fx = ft_input->get(0).asDouble();
	_fy = ft_input->get(1).asDouble();
	_fz = ft_input->get(2).asDouble();
	_tx = ft_input->get(3).asDouble();
	_ty = ft_input->get(4).asDouble();
	_tz = ft_input->get(5).asDouble();

	array<double, 6> tempArray;
	tempArray.at(0) = _fx - _fxBias;
	tempArray.at(1) = _fy - _fyBias;
	tempArray.at(2) = _fz - _fzBias;
	tempArray.at(3) = _tx - _txBias;
	tempArray.at(4) = _ty - _tyBias;
	tempArray.at(5) = _tz - _tzBias;

	array<double, 6> prevData = _filterBuffer.front();
	_filterBuffer.pop();
	_filterBuffer.push(tempArray);

	_fxFiltered += (_fx -_fxBias - prevData.at(0))/FILTER_WINDOW;
	_fyFiltered += (_fy -_fyBias - prevData.at(1))/FILTER_WINDOW;
	_fzFiltered += (_fz -_fzBias - prevData.at(2))/FILTER_WINDOW;

	_mutex.unlock();

	return true;

}

void forceTorqueData::setForces(double fx, double fy, double fz)
{
	_mutex.lock();
	_fx = fx; _fy = fy; _fz = fz;
	_mutex.unlock();
}
void  forceTorqueData::setTorques(double tx, double ty, double tz)
{
	_mutex.lock();
	_tx = tx; _ty = ty; _tz = tz;
	_mutex.unlock();
}

void forceTorqueData::setBias()
{
	_mutex.lock();
	_fxBias = _fx; _fyBias = _fy; _fzBias = _fz;
	_mutex.unlock();
	// Reset the filter buffer to zero
	this->resetFilterBuffer();
}
void forceTorqueData::setBias(double fxBias, double fyBias, double fzBias)
{
	_mutex.lock();
	_fxBias = fxBias; _fyBias = fyBias; _fzBias = fzBias;
	_mutex.unlock();

	// Reset the filter buffer to zero
	this->resetFilterBuffer();
}

void forceTorqueData::getForces(double *fx, double *fy, double *fz)
{
	_mutex.lock();
	*fx = _fx; *fy = _fy; *fz = _fz;
	_mutex.unlock();

}