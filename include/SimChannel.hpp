#ifndef SIM_EEROS_CHANNEL_HPP_
#define SIM_EEROS_CHANNEL_HPP_

#include <atomic>
#include <iostream>

namespace sim {
	
	template <typename T>
	class SimChannel {
		public:
			SimChannel(int subDevice, int channel) {this->subDevice = subDevice; this->channel = channel;}
			~SimChannel() {}
			T getValue() { return value.load(); }
			void setValue(T val) { value.store(val); }
			
			int getSubDevice() { return subDevice; }
			int getChannel() { return channel; }
			
		private:
			int subDevice;
			int channel;
			std::atomic<T> value {0};
	};
};

#endif /* #ifndef SIM_EEROS_CHANNEL_HPP_ */