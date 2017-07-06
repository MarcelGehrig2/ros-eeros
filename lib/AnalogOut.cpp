#include "../include/AnalogOut.hpp"
#include <iostream>

using namespace sim;

AnalogOut::AnalogOut(std::string id,
					 void* libHandle,
					 std::string device,
					 uint32_t subDeviceNumber,
					 uint32_t channel,
					 double scale,
					 double offset,
					 double rangeMin,
					 double rangeMax,
					 std::string unit
		    ) : ScalableOutput<double>(id, libHandle, scale, offset, rangeMin, rangeMax, unit) {
	SimDevice *dev = SimDevice::getDevice(device);
	this->chan = dev->getRealChannel(subDeviceNumber, channel);
}

double AnalogOut::get() {
	return (chan->getValue() - offset) / scale;
	std::cout << "ros-eeros get: " << chan->getValue() << std::endl;
}

void AnalogOut::set(double voltage) {
	double value = (voltage - offset)/scale;
	
	if(value > maxOut) value = maxOut;
	if(value < minOut) value = minOut;
	
	chan->setValue(value);
// 	std::cout << "ros-eeros set: " << chan->getValue() << std::endl;
}

extern "C"{
	eeros::hal::ScalableOutput<double> *createAnalogOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, 
							    double scale, double offset, double rangeMin, double rangeMax, std::string unit){
		return new sim::AnalogOut(id, libHandle, device, subDeviceNumber, channel, scale, offset, rangeMin, rangeMax, unit);
	}
}
