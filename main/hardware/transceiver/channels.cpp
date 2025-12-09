#include "channels.h"

#include "aircraft.h"

namespace pizda {
	void Channels::setup() {
		updateFromSettings();
	}

	void Channels::updateFromSettings() {
		auto& ac = Aircraft::getInstance();

		for (auto& channel : instances) {
			if (channel) {
				delete channel;
				channel = nullptr;
			}
		}

		Channel* channel = nullptr;
		size_t channelIndex = 0;

		for (int i = 0; i < ac.settings.channelDataStructure.fields.size(); ++i) {
			auto& field = ac.settings.channelDataStructure.fields[i];

			for (int j = 0; j < field.count; ++j) {
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

	bool Channels::checkChannel(uint8_t channelIndex, ChannelDataType dataType) {
		auto channel = instances[channelIndex];

		if (!channel) {
			ESP_LOGE("Channels", "check failed, channel %d is null", channelIndex);
			return false;
		}
		else if (channel->getDataType() != dataType) {
			ESP_LOGE("Channels", "check failed, channel %d data type %d != requested data type", channelIndex, channel->getDataType(), dataType);
			return false;
		}

		return true;
	}

	UintChannel* Channels::checkUintChannel(uint8_t channelIndex, uint8_t bitDepth) {
		if (!checkChannel(channelIndex, ChannelDataType::unsignedInteger)) {
			return nullptr;
		}

		const auto uintChannel = reinterpret_cast<UintChannel*>(instances[channelIndex]);

		if (uintChannel->getBitDepth() != bitDepth) {
			ESP_LOGE("Channels", "check failed, channel %d bit depth %d != requested bit depth", channelIndex, uintChannel->getBitDepth(), bitDepth);
			return nullptr;
		}

		return uintChannel;
	}

	BoolChannel* Channels::checkBoolChannel(uint8_t channelIndex) {
		if (!checkChannel(channelIndex, ChannelDataType::boolean))
			return nullptr;

		return reinterpret_cast<BoolChannel*>(instances[channelIndex]);
	}

	void Channels::checkAndSetMotor(uint8_t channelIndex, uint8_t motorIndex) {
		const auto uintChannel = checkUintChannel(channelIndex, Motor::powerBitCount);

		if (!uintChannel)
			return;

		Aircraft::getInstance().motors.setPower(motorIndex, uintChannel->getValue());
	}

	void Channels::onValueUpdated() {
		auto& ac = Aircraft::getInstance();

		// Motors
		checkAndSetMotor(2, 2);
		checkAndSetMotor(5, 6);

		// Lights
		{
			BoolChannel* boolChannel;

			if ((boolChannel = checkBoolChannel(6)))
				ac.lights.setNavigationEnabled(boolChannel->getValue());

			if ((boolChannel = checkBoolChannel(7)))
				ac.lights.setStrobeEnabled(boolChannel->getValue());

			if ((boolChannel = checkBoolChannel(8)))
				ac.lights.setLandingEnabled(boolChannel->getValue());

			if ((boolChannel = checkBoolChannel(9)))
				ac.lights.setCabinEnabled(boolChannel->getValue());
		}
	}
}