#ifndef SIM_EEROS_REFLECT_HPP_
#define SIM_EEROS_REFLECT_HPP_

#include <vector>
#include <memory>

#include "SimBehaviour.hpp"
#include "SimChannel.hpp"
#include <eeros/core/Fault.hpp>

/**
 * usage: 
 * simulate digital Output:
 * - connect digital output from eeros-application to subdevice 0 (REFLECT_OUT_DIGOUT) and desired channel x
 * - connect digital test input to subdevice 1 (REFLECT_OUT_DIGIN) and channel x
 * - you can read state of digital output at your test input channel
 * 
 * simulate digital Input:
 * - connect intput to subdevice 2 (REFLECT_IN_DIGIN) and channel y
 * - connect test output to subdevice 3 (REFLECT_IN_DIGOUT) and channel y
 * - read state of your input on test output channel
 */

namespace sim{

	template <typename T>
	class Reflect : public SimBehaviour<T> {
		public:
			Reflect(int nofSimChannels, std::initializer_list<int> subDeviceNumber) : subDevNumbers(subDeviceNumber){
				if(subDeviceNumber.size() != 2){
					throw eeros::Fault("number of SubDeviceNumbers is not 2!");
				}
				// create channels
				for(int i = 0; i < nofSimChannels; i++){
					in.push_back(std::shared_ptr<SimChannel<T>>(new SimChannel<T>(subDevNumbers[0], i)));
					out.push_back(std::shared_ptr<SimChannel<T>>(new SimChannel<T>(subDevNumbers[1], i)));
				}
			}
			~Reflect() {
				in.clear();
				out.clear();
				subDevNumbers.clear();
			}
			virtual void run(){
				// reflect
				for(int i = 0; i < in.size(); i++){
					out[i]->setValue(in[i]->getValue());
				}
			}
			
			virtual std::shared_ptr<sim::SimChannel<T>> getInChannel(int channel){
				for(int i = 0; i < in.size(); i++){
					if(in[i]->getChannel() == channel){
						return in[i];
					}
				}
				throw eeros::Fault("In channel not found");
			}
			virtual std::shared_ptr<sim::SimChannel<T>> getOutChannel(int channel){
				for(int i = 0; i < out.size(); i++){
					if(out[i]->getChannel() == channel){
						return out[i];
					}
				}
				throw eeros::Fault("Out channel not found");
			}
			
		private:
			// simulation: reflect in to out
			std::vector<std::shared_ptr<sim::SimChannel<T>>> out;
			std::vector<std::shared_ptr<sim::SimChannel<T>>> in;
			
			std::vector<int> subDevNumbers;
  
	};
};

#endif /* SIM_EEROS_REFLECT_HPP_ */