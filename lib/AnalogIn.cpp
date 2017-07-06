
#include "../include/AnalogIn.hpp"
#include <ros/ros.h>
#include <ros/callback_queue.h>

using namespace halros;

AnalogIn::AnalogIn(std::string id,
					 void* libHandle,
					 std::string device,
					 uint32_t subDeviceNumber,
					 uint32_t channel,
					 double scale,
					 double offset,
					 double rangeMin,
					 double rangeMax,
					 std::string unit
		  ) : ScalableInput<double>(id, libHandle, scale, offset, rangeMin, rangeMax, unit),
		      subDeviceNumber(subDeviceNumber), channel(channel) {
	dev = RosNodeDevice::getDevice(device);
	
	int queueSize = 1000;
	rosNodeHandle= dev->getRosNodeHandle();
	subscriber = rosNodeHandle->subscribe(id, queueSize, &AnalogIn::rosCallbackFct, this);
}

void AnalogIn::rosCallbackFct(const std_msgs::Float64::Type& msg) {
	data = msg.data;
}

double AnalogIn::get() {
//  	ros::getGlobalCallbackQueue()->callAvailable();		// calls callback fct. for all available messages.
														//  Only newest message is processed. Older ones are discarded.
	ros::getGlobalCallbackQueue()->callOne();			// calls callback fct. only for the oldest message
	
	double inVal = (data - offset) / scale;
	
	if(inVal > maxIn) inVal = maxIn;
	if(inVal < minIn) inVal = minIn;
	
	return inVal;
}

extern "C"{
	eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, 
							void* libHandle, 
							std::string device, 
							uint32_t subDeviceNumber, 
							uint32_t channel, 
							double scale, 
							double offset, 
							double rangeMin, 
							double rangeMax, 
							std::string unit){
		return new halros::AnalogIn(id, libHandle, device, subDeviceNumber, channel, scale, offset, rangeMin, rangeMax, unit);
	}
}

