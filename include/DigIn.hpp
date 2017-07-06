#ifndef SIM_EEROS_DIGIN_HPP_
#define SIM_EEROS_DIGIN_HPP_

#include <string>
#include <memory>
#include <eeros/hal/Input.hpp>
#include "SimDevice.hpp"
#include "SimChannel.hpp"

namespace sim {
	class DigIn : public eeros::hal::Input<bool> {
	public:
		DigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted = false);
		virtual bool get();
		
	private:
		std::shared_ptr<SimChannel<bool>> chan;
		
		bool inverted;
		
	};
};

extern "C"{
	eeros::hal::Input<bool> *createDigIn(std::string id, void* libHandle, std::string device, uint32_t subDeviceNumber, uint32_t channel, bool inverted);
}

#endif /* SIM_EEROS_DIGIN_HPP_ */
