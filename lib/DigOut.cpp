#include "../include/DigOut.hpp"

using namespace sim;

DigOut::DigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted) : 
					Output<bool>(id, libHandle), inverted(inverted) {
	SimDevice *dev = SimDevice::getDevice(device);
	chan = dev->getLogicChannel(subDeviceNumber, channel);
}

bool DigOut::get() {
	if(inverted) return !(chan->getValue());
	return chan->getValue();
}

void DigOut::set(bool value) {
	if(inverted) value = !value;
	chan->setValue(value);
}

extern "C" eeros::hal::Output<bool> *createDigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted){
	return new sim::DigOut(id, libHandle, device, subDeviceNumber, channel, inverted);
}
