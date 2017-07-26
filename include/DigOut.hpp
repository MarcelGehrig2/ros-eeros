#ifndef ROS_EEROS_DIGOUT_HPP_
#define ROS_EEROS_DIGOUT_HPP_

#include <string>
#include <eeros/hal/Output.hpp>
#include "RosNodeDevice.hpp"
#include <ros/ros.h>
// ROS message types
#include <sensor_msgs/BatteryState.h>

namespace halros {
	class DigOut : public eeros::hal::Output<bool> {
	public:
		DigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
			   bool inverted = false, std::string additionalArguments = "");
		
		virtual bool get();
		virtual void set(bool value);
		
		
	private:
		void (*setFunction) (const bool, const ros::Publisher&);
		static ros::Time getTime();
		
		// set functions for ROS
		// /////////////////////
		static void sensorMsgsBatteryStatePresent		(const bool value, const ros::Publisher& publisher);
		
		RosNodeDevice* dev;
		std::shared_ptr<ros::NodeHandle> rosNodeHandle;
		ros::Subscriber subscriber;
		uint32_t subDeviceNumber;
		uint32_t channel;
		ros::Publisher publisher;
		bool data; 
		std::string msgType;
		std::string topic;
		std::string dataField;
		int queueSize;
		bool callOne;
		
		bool inverted;
	};
};

extern "C"{
	eeros::hal::Output<bool> *createDigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted, std::string additionalArguments);
}

#endif /* ROS_EEROS_DIGOUT_HPP_ */
