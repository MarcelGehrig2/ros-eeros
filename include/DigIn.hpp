#ifndef ROS_EEROS_DIGIN_HPP_
#define ROS_EEROS_DIGIN_HPP_

#include <string>
#include <memory>
#include <eeros/hal/Input.hpp>
#include "RosNodeDevice.hpp"
#include <ros/ros.h>
// ROS message types
#include <sensor_msgs/BatteryState.h>

namespace halros {
	class DigIn : public eeros::hal::Input<bool> {
	public:
		DigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted = false, std::string additionalArguments = "");
		virtual bool get();
		
	private:
		// callback functions for ROS
		void sensorMsgsBatteryStatePresent		(const sensor_msgs::BatteryState::Type& msg)	{data = msg.present;} ;
		
		
		bool inverted;
		RosNodeDevice* dev;
		std::shared_ptr<ros::NodeHandle> rosNodeHandle;
		ros::Subscriber subscriber;
		uint32_t subDeviceNumber;
		uint32_t channel;
		bool data; 
		std::string msgType;
		std::string topic;
		std::string dataField;
		int queueSize;
		bool callOne;
		
	};
};

extern "C"{
	eeros::hal::Input<bool> *createDigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted, std::string additionalArguments);
}

#endif /* ROS_EEROS_DIGIN_HPP_ */
