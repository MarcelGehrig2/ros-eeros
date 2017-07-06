#ifndef SIM_EEROS_ANALOGIN_HPP_
#define SIM_EEROS_ANALOGIN_HPP_

#include <string>
// #include <limits>
// #include <comedilib.h>
#include <eeros/hal/ScalableInput.hpp>
#include "RosNodeDevice.hpp"
#include <std_msgs/Float64.h>

namespace halros {
	class AnalogIn : public eeros::hal::ScalableInput<double> {
	public:
		AnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, double scale = 1, double offset = 0, double rangeMin = std::numeric_limits<double>::min(), double rangeMax = std::numeric_limits<double>::max(), std::string unit = "");
		virtual double get();
		
		void rosCallbackFct(const std_msgs::Float64::Type& msg);
		
		
	private:
		RosNodeDevice* dev;
		std::shared_ptr<ros::NodeHandle> rosNodeHandle;
		ros::Subscriber subscriber;
		uint32_t subDeviceNumber;
		uint32_t channel;
		double data; 
	};
};


extern "C"{
	eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, double scale, double offset, double rangeMin, double rangeMax, std::string unit);
}


#endif /* SIM_EEROS_ANALOGIN_HPP_ */