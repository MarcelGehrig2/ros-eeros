#include "../include/SimDevice.hpp"
#include <eeros/core/Fault.hpp>
#include <unistd.h>
#include <iostream>

using namespace sim;

std::map<std::string, SimDevice *> SimDevice::devices;

#define NOF_SIM_CHANNELS 10

// device will select function 

SimDevice::SimDevice(std::string simId, int nofSimChannels, std::initializer_list<int> subDevNumDigOut, std::initializer_list<int> subDevNumDigIn, 
		     std::initializer_list<int> subDevNumAnalogOut, std::initializer_list<int> subDevNumAnalogIn) :
		      digOut(nofSimChannels, subDevNumDigOut),
		      digIn(nofSimChannels, subDevNumDigIn),
		      analogOut(nofSimChannels, subDevNumAnalogOut),
		      analogIn(nofSimChannels, subDevNumAnalogIn) {
	this->simId = simId;
	
	logicSimBlocks.push_back(&digOut);
	logicSimBlocks.push_back(&digIn);
	scalableSimBlocks.push_back(&analogOut);
	scalableSimBlocks.push_back(&analogIn);
	
	auto devIt = devices.find(simId);
	if(devIt != devices.end()){
		throw new eeros::Fault("device already open, claim already opened device via getDevice()"); // should not occur!
	}
	
	t = new std::thread([this](){ this->run(); });
	
	devices[simId] = this;
}

SimDevice::~SimDevice() {
	auto devIt = devices.find(simId);
	devices.erase(devIt);
	
	delete t;
}

SimDevice* SimDevice::getDevice(std::string simId) {
	std::cout << "SimDevice.cpp getDevice simID: " << simId << std::endl;
	auto devIt = devices.find(simId);
	if(devIt != devices.end()){
		return devIt->second;
	}
	else{
		for(int i = 0; i < simFeatures.size(); i++){
			if(simFeatures[i] == simId){
				if(simId == "reflect"){
					return new SimDevice(simId, NOF_SIM_CHANNELS, {REFLECT_OUT_DIGOUT, REFLECT_OUT_DIGIN}, {REFLECT_IN_DIGIN, REFLECT_IN_DIGOUT}, 
							     {REFLECT_OUT_AOUT, REFLECT_OUT_AIN}, {REFLECT_IN_AIN, REFLECT_IN_AOUT});
				}
			}
		}
		throw eeros::Fault("simulation feature '" + simId + "' is not supported.");
	}
}

std::shared_ptr<SimChannel<bool>> SimDevice::getLogicChannel(int subDeviceNumber, int channel) {
	if(simId == "reflect"){
		// digital output simulation block
		
		switch(subDeviceNumber){
			// simulate digital Out
			case REFLECT_OUT_DIGOUT:{
				return digOut.getInChannel(channel);
				break;		// not reached
			}
			case REFLECT_OUT_DIGIN:{
				return digOut.getOutChannel(channel);
				break;		// not reached
			}
			// simulate digital In
			case REFLECT_IN_DIGIN:{
				return digIn.getOutChannel(channel);
				break;
			}
			case REFLECT_IN_DIGOUT:{
				return digIn.getInChannel(channel);
				break;
			}
		}
	}
	else{
		throw eeros::Fault("getChannel failed: no such device");
	}
}

std::shared_ptr<SimChannel<double>> SimDevice::getRealChannel(int subDeviceNumber, int channel) {
	if(simId == "reflect"){
		// digital output simulation block
		
		switch(subDeviceNumber){
			// simulate analog Out
			case REFLECT_OUT_AOUT:{
				return analogOut.getInChannel(channel);
				break;		// not reached
			}
			case REFLECT_OUT_AIN:{
				return analogOut.getOutChannel(channel);
				break;		// not reached
			}
			// simulate analog In
			case REFLECT_IN_AIN:{
				return analogIn.getOutChannel(channel);
				break;
			}
			case REFLECT_IN_AOUT:{
				return analogIn.getInChannel(channel);
				break;
			}
		}
	}
	else{
		throw eeros::Fault("getChannel failed: no such device");
	}
}

void SimDevice::run() {
	while(true){
		
		for(int i = 0; i < logicSimBlocks.size(); i++) {
			logicSimBlocks[i]->run();
		}
		for(int i = 0; i < scalableSimBlocks.size(); i++) {
			scalableSimBlocks[i]->run();
		}
		
		usleep(1000);
	}
}

