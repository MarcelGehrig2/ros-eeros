#ifndef SIM_EEROS_DIGOUT_HPP_
#define SIM_EEROS_DIGOUT_HPP_

#include <string>
#include <memory>
#include <eeros/hal/Output.hpp>
#include "SimDevice.hpp"
#include "SimChannel.hpp"

namespace sim {
	class DigOut : public eeros::hal::Output<bool> {
	public:
		DigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted = false);
		virtual bool get();
		virtual void set(bool value);
		
	private:
		std::shared_ptr<SimChannel<bool>> chan;
		
		bool inverted;
	};
};

extern "C"{
	eeros::hal::Output<bool> *createDigOut(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted);
}

#endif /* SIM_EEROS_DIGOUT_HPP_ */
