#include "forceTorqueData.h"
#include <yarp/os/Network.h>

ForceTorqueData::ForceTorqueData()
{

	_fx = _fy = _fz =0;
	_fxBias = _fyBias = _fzBias = 0;
	_fxFiltered = _fyFiltered = _fzFiltered = 0;
	_txFiltered = _tyFiltered = _tzFiltered = 0;

	_timeStamp.update();

	_port_ft.open("/OmegaATI/ft");
	_port_ftFiltered.open("/OmegaATI/ftFiltered");

	yarp::os::Network::connect("/NIDAQmxReader/data/real:o","/OmegaATI/ft");

	array<double, FT_CHANNELS> tempArray;
	tempArray.fill(0);
	for (int i = 0; i < FILTER_WINDOW; i++)
		_filterBuffer.push(tempArray);
};

void ForceTorqueData::publishData()
{
	
	Bottle& ft_output = _port_ft.prepare();
	ft_output.clear();

	_mutex.lock();
	ft_output.addDouble(_fx - _fxBias);
	ft_output.addDouble(_fy - _fyBias);
	ft_output.addDouble(_fz - _fzBias);
	_port_ft.setEnvelope(_timeStamp);
	_mutex.unlock(); //////////////////<<<<--
    _port_ft.write();

		/*

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


}
bool ForceTorqueData::updateData()
{

	Bottle *ft_input = _port_ft.read(); //  Read f/t data 

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

void ForceTorqueData::setForces(double fx, double fy, double fz)
{
	_mutex.lock();
	_fx = fx; _fy = fy; _fz = fz;
	_mutex.unlock();
}
void  ForceTorqueData::setTorques(double tx, double ty, double tz)
{
	_mutex.lock();
	_tx = tx; _ty = ty; _tz = tz;
	_mutex.unlock();
}

void ForceTorqueData::setBias()
{
	_mutex.lock();
	_fxBias = _fx; _fyBias = _fy; _fzBias = _fz;
	_mutex.unlock();
	// Reset the filter buffer to zero
	this->resetFilterBuffer();
}
void ForceTorqueData::setBias(double fxBias, double fyBias, double fzBias)
{
	_mutex.lock();
	_fxBias = fxBias; _fyBias = fyBias; _fzBias = fzBias;
	_mutex.unlock();

	// Reset the filter buffer to zero
	this->resetFilterBuffer();
}

void ForceTorqueData::getForces(double *fx, double *fy, double *fz)
{
	_mutex.lock();
	*fx = _fx; *fy = _fy; *fz = _fz;
	_mutex.unlock();

}