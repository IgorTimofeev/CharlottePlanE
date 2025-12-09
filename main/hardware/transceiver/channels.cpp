#include "channels.h"

#include "aircraft.h"

namespace pizda {
	void Channels::setup() {
		updateFromDataStructure();
	}

	void Channels::updateFromDataStructure() {
		auto& ac = Aircraft::getInstance();

		// Deleting existing channels
		for (auto& channel : instances) {
			if (channel) {
				delete channel;
				channel = nullptr;
			}
		}

		// Creating new channels
		Channel* channel = nullptr;

		uint16_t channelIndex = 0;

		for (const auto& field : ac.settings.channelDataStructure.fields) {
			for (uint16_t i = 0; i < field.count; ++i) {
				if (channelIndex >= instances.size()) {
					ESP_LOGE("Channels", "updateFromDataStructure() failed, channel %d >= channels size %d", channelIndex, instances.size());
					return;
				}

				switch (field.type) {
					case ChannelDataType::unsignedInteger:
						channel = new UintChannel(field.bitDepth);
						break;

					case ChannelDataType::boolean:
						channel = new BoolChannel();
						break;
				}

				instances[channelIndex] = channel;

				channelIndex++;
			}
		}
	}

	Channel* Channels::getChannel(uint8_t channelIndex) {
		if (channelIndex >= instances.size()) {
			ESP_LOGE("Channels", "getChannel() failed, channel %d >= channels size %d", channelIndex, instances.size());
			return nullptr;
		}

		const auto channel = instances[channelIndex];

		if (!channel) {
			ESP_LOGE("Channels", "getChannel() failed, channel %d is not configured", channelIndex);
			return nullptr;
		}

		return channel;
	}

	Channel* Channels::getChannelAndCheckDataType(uint8_t channelIndex, ChannelDataType dataType) {
		const auto channel = getChannel(channelIndex);

		if (channel->getDataType() != dataType) {
			ESP_LOGE("Channels", "getChannelAndCheckDataType() failed, channel %d data type %d != requested data type", channelIndex, channel->getDataType(), dataType);
			return nullptr;
		}

		return channel;
	}

	UintChannel* Channels::getUintChannel(uint8_t channelIndex, uint8_t bitDepth) {
		const auto channel = getChannelAndCheckDataType(channelIndex, ChannelDataType::unsignedInteger);

		if (!channel)
			return nullptr;

		const auto uintChannel = reinterpret_cast<UintChannel*>(channel);

		if (uintChannel->getBitDepth() != bitDepth) {
			ESP_LOGE("Channels", "getUintChannel() failed, channel %d bit depth %d != requested bit depth", channelIndex, uintChannel->getBitDepth(), bitDepth);
			return nullptr;
		}

		return uintChannel;
	}

	BoolChannel* Channels::getBoolChannel(uint8_t channelIndex) {
		const auto channel = getChannelAndCheckDataType(channelIndex, ChannelDataType::boolean);

		return channel ? reinterpret_cast<BoolChannel*>(channel) : nullptr;
	}

}