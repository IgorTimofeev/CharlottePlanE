#pragma once

#include <array>
#include <optional>
#include <esp_log.h>

namespace pizda {
	class ChannelBinding {
		public:
			virtual void onChannelValueChanged(uint32_t value) {
				ESP_LOGE("ChannelBinding", "onChannelValueChanged(uint32_t value) is not implemented");
			}

			virtual void onChannelValueChanged(bool value) {
				ESP_LOGE("ChannelBinding", "onChannelValueChanged(bool value) is not implemented");
			}
	};

	class Channels {
		public:
			Channels() {
				for (auto& binding : bindings) {
					binding = nullptr;
				}
			}

			void setBinding(uint8_t channelIndex, ChannelBinding* binding) {
				bindings[channelIndex] = binding;
			}

			void setValue(uint8_t channelIndex, uint32_t value) {
				setValue<uint32_t>(channelIndex, value);
			}

			void setValue(uint8_t channelIndex, bool value) {
				setValue<bool>(channelIndex, value);
			}

		private:
			std::array<ChannelBinding*, 255> bindings {};

			template<typename TValue>
			void setValue(uint8_t channelIndex, TValue value) {
				if (bindings[channelIndex]) {
					bindings[channelIndex]->onChannelValueChanged(value);
				}
				else {
					ESP_LOGE("Channels", "channel %d is not bound", channelIndex);
				}
			}
	};
}