#pragma once

#include <array>
#include <optional>
#include <esp_log.h>

namespace pizda {
	class ChannelAware {
		public:
			virtual void fromChannel(uint32_t value) {
				ESP_LOGE("ChannelAware", "fromChannel(uint32_t value) is not implemented");
			}

			virtual void fromChannel(bool value) {
				ESP_LOGE("ChannelAware", "fromChannel(bool value) is not implemented");
			}
	};

	class Channels {
		public:
			Channels() {
				for (auto& binding : bindings) {
					binding = nullptr;
				}
			}

			std::array<ChannelAware*, 255> bindings {};

			void setValue(uint8_t channelIndex, uint32_t value) {
				setValue<uint32_t>(channelIndex, value);
			}

			void setValue(uint8_t channelIndex, bool value) {
				setValue<bool>(channelIndex, value);
			}

		private:
			template<typename TValue>
			void setValue(uint8_t channelIndex, TValue value) {
				if (bindings[channelIndex]) {
					bindings[channelIndex]->fromChannel(value);
				}
				else {
					ESP_LOGE("Channels", "channel with index %d is empty", channelIndex);
				}
			}
	};
}