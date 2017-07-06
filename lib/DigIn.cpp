#include "../include/DigIn.hpp"

using namespace sim;

DigIn::DigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted) : 
				Input<bool>(id, libHandle), inverted(inverted) {
	SimDevice *dev = SimDevice::getDevice(device);
	this->chan = dev->getLogicChannel(subDeviceNumber, channel);
}

bool DigIn::get() {
	if(inverted) return !(chan->getValue());
	return chan->getValue();
}

extern "C" eeros::hal::Input<bool> *createDigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted){
	return new sim::DigIn(id, libHandle, device, subDeviceNumber, channel, inverted);
}