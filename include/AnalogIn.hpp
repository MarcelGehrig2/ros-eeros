#ifndef SIM_EEROS_ANALOGIN_HPP_
#define SIM_EEROS_ANALOGIN_HPP_

#include <string>
// #include <limits>
// #include <comedilib.h>
#include <eeros/hal/ScalableInput.hpp>
#include "RosNodeDevice.hpp"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

namespace halros {
	class AnalogIn : public eeros::hal::ScalableInput<double> {
	public:
		AnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, double scale = 1, double offset = 0, double rangeMin = std::numeric_limits<double>::min(), double rangeMax = std::numeric_limits<double>::max(), std::string unit = "", std::string additionalArguments = "");
		virtual double get();
		
		// callback functions for ROS
		void stdMsgsFloat64Data					(const std_msgs::Float64::Type& msg) 		{data = msg.data;} ;
		
		void sensorMsgsLaserScanAngleMin		( const sensor_msgs::LaserScan::Type& msg)	{data = msg.angle_min;} ;
		void sensorMsgsLaserScanAngleMax		( const sensor_msgs::LaserScan::Type& msg)	{data = msg.angle_max;} ;
		void sensorMsgsLaserScanAngleIncrement	( const sensor_msgs::LaserScan::Type& msg)	{data = msg.angle_increment;} ;
		void sensorMsgsLaserScanTimeIncrement	( const sensor_msgs::LaserScan::Type& msg)	{data = msg.time_increment;} ;
		void sensorMsgsLaserScanScanTime		( const sensor_msgs::LaserScan::Type& msg)	{data = msg.scan_time;} ;
		void sensorMsgsLaserScanRangeMin		( const sensor_msgs::LaserScan::Type& msg)	{data = msg.range_min;} ;
		void sensorMsgsLaserScanRangeMax		( const sensor_msgs::LaserScan::Type& msg)	{data = msg.range_max;} ;
			
		
	private:
		RosNodeDevice* dev;
		std::shared_ptr<ros::NodeHandle> rosNodeHandle;
		ros::Subscriber subscriber;
		uint32_t subDeviceNumber;
		uint32_t channel;
		double data; 
		std::string msgType;
		std::string topic;
		std::string dataMember;
		int queueSize;
		bool callOne;
	};
};


extern "C"{
	eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, double scale, double offset, double rangeMin, double rangeMax, std::string unit, std::string additionalArguments);
}


#endif /* SIM_EEROS_ANALOGIN_HPP_ */