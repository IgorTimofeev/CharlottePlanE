#include "channels.h"

#include <utility>

#include "aircraft.h"

namespace pizda {
	void Channels::setup() {
		updateFromDataStructure();
	}

	void Channels::updateFromDataStructure() {
		auto& ac = Aircraft::getInstance();

		// Deleting existing channels
		for (auto channel : _instances) {
			if (channel) {
				delete channel;
			}
		}
		
		_instances.clear();
		
		// TMP
		_instances.push_back(new UintChannel(RemoteChannelsPacket::motorLengthBits));
		_instances.push_back(new UintChannel(RemoteChannelsPacket::motorLengthBits));
		_instances.push_back(new UintChannel(RemoteChannelsPacket::motorLengthBits));
		_instances.push_back(new UintChannel(RemoteChannelsPacket::motorLengthBits));
		_instances.push_back(new UintChannel(RemoteChannelsPacket::motorLengthBits));
		_instances.push_back(new UintChannel(RemoteChannelsPacket::motorLengthBits));
		
		_instances.push_back(new BoolChannel());
		_instances.push_back(new BoolChannel());
		_instances.push_back(new BoolChannel());
		_instances.push_back(new BoolChannel());
		
		return;

		// Creating new channels
		Channel* channel = nullptr;

		uint16_t channelIndex = 0;

		for (const auto& field : ac.settings.channelDataStructure.fields) {
			for (uint16_t i = 0; i < field.count; ++i) {
				switch (field.type) {
					case ChannelDataType::unsignedInteger:
						channel = new UintChannel(field.bitDepth);
						break;

					case ChannelDataType::boolean:
						channel = new BoolChannel();
						break;
				}

				_instances.push_back(channel);

				channelIndex++;
			}
		}
	}

	Channel* Channels::getChannel(uint8_t channelIndex) {
		if (channelIndex >= _instances.size()) {
			ESP_LOGE(_logTag, "getChannel() failed, channel %d >= channels size %d", channelIndex, _instances.size());
			return nullptr;
		}
		
		return _instances[channelIndex];
	}

	Channel* Channels::getChannel(ChannelType channelType) {
		return getChannel(std::to_underlying(channelType));
	}

	Channel* Channels::getChannelAndCheckDataType(uint8_t channelIndex, ChannelDataType dataType) {
		const auto channel = getChannel(channelIndex);

		if (channel->getDataType() != dataType) {
			ESP_LOGE(_logTag, "getChannelAndCheckDataType() failed, channel %d data type %d != requested data type", channelIndex, channel->getDataType(), dataType);
			return nullptr;
		}

		return channel;
	}

	UintChannel* Channels::getUintChannel(uint8_t channelIndex) {
		const auto channel = getChannelAndCheckDataType(channelIndex, ChannelDataType::unsignedInteger);

		return channel ? reinterpret_cast<UintChannel*>(channel) : nullptr;
	}

	UintChannel* Channels::getUintChannel(ChannelType channelType) {
		return getUintChannel(std::to_underlying(channelType));
	}

	BoolChannel* Channels::getBoolChannel(uint8_t channelIndex) {
		const auto channel = getChannelAndCheckDataType(channelIndex, ChannelDataType::boolean);

		return channel ? reinterpret_cast<BoolChannel*>(channel) : nullptr;
	}

	BoolChannel* Channels::getBoolChannel(ChannelType channelType) {
		return getBoolChannel(std::to_underlying(channelType));
	}
}