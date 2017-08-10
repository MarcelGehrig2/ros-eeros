#ifndef ROS_EEROS_ANALOGIN_HPP_
#define ROS_EEROS_ANALOGIN_HPP_

#include <eeros/hal/ScalableInput.hpp>
#include "RosNodeDevice.hpp"
#include <ros/ros.h>
// 1.) Include ROS message type
// ////////////////////////////
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>

namespace halros {
	class AnalogIn : public eeros::hal::ScalableInput<double> {
	public:
		AnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
				 double scale = 1, double offset = 0,
		   double rangeMin = std::numeric_limits<double>::min(), double rangeMax = std::numeric_limits<double>::max(), std::string unit = "",
				 std::string additionalArguments = "");
		
		virtual double get();
		
		
	private:
		// 2.) Create new callback functions for ROS
		// /////////////////////////////////////////
		void stdMsgsFloat64Data					(const std_msgs::Float64::Type& msg) 		{data = msg.data;} ;
		
		void sensorMsgsLaserScanAngleMin		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.angle_min;} ;
		void sensorMsgsLaserScanAngleMax		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.angle_max;} ;
		void sensorMsgsLaserScanAngleIncrement	(const sensor_msgs::LaserScan::Type& msg)	{data = msg.angle_increment;} ;
		void sensorMsgsLaserScanTimeIncrement	(const sensor_msgs::LaserScan::Type& msg)	{data = msg.time_increment;} ;
		void sensorMsgsLaserScanScanTime		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.scan_time;} ;
		void sensorMsgsLaserScanRangeMin		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.range_min;} ;
		void sensorMsgsLaserScanRangeMax		(const sensor_msgs::LaserScan::Type& msg)	{data = msg.range_max;} ;
		
		void sensorMsgsJointStatePosition0		(const sensor_msgs::JointState::Type& msg)	{data = msg.position[0];} ;


		RosNodeDevice* dev;
		std::shared_ptr<ros::NodeHandle> rosNodeHandle;
		ros::Subscriber subscriber;
		uint32_t subDeviceNumber;
		uint32_t channel;
		double data; 
		std::string msgType;
		std::string topic;
		std::string dataField;
		int queueSize;
		bool callOne;
	};
};


extern "C"{
	eeros::hal::ScalableInput<double> *createAnalogIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel,
														double scale, double offset, double rangeMin, double rangeMax, std::string unit,
														std::string additionalArguments);
}


#endif /* ROS_EEROS_ANALOGIN_HPP_ */