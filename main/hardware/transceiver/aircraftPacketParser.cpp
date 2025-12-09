#include "aircraftPacketParser.h"

#include "aircraft.h"
#include "hardware/motor.h"

namespace pizda {
	bool AircraftPacketParser::onParse(ReadableBitStream& stream, PacketType packetType) {
		auto& ac = Aircraft::getInstance();

		switch (packetType) {
			case PacketType::RemoteChannelDataStructure: {
				const auto valueCount = stream.readUint8(8);

				ESP_LOGI("PacketParser", "data type count: %d", valueCount);

				ac.settings.channelDataStructure.fields.clear();
				ac.settings.channelDataStructure.fields.reserve(valueCount);

				ChannelDataStructureSettingsField field {};

				for (uint8_t i = 0; i < valueCount; ++i) {
					field.type = static_cast<ChannelDataType>(stream.readUint8(3));

					switch (field.type) {
						case ChannelDataType::unsignedInteger: {
							field.bitDepth = stream.readUint8(5);
							field.count = stream.readUint8(8);

							ESP_LOGI("PacketParser", "data type #%d, type: uint, bit depth: %d, count: %d", i, field.bitDepth, field.count);

							break;
						}
						case ChannelDataType::boolean: {
							field.bitDepth = 1;
							field.count = stream.readUint8(8);

							ESP_LOGI("PacketParser", "data type #%d, type: bool", i);

							break;
						}
						default: {
							ESP_LOGI("PacketParser", "unknown data type");
							return false;
						}
					}

					ac.settings.channelDataStructure.fields.push_back(field);
				}

				ac.channels.updateFromSettings();
				ac.settings.channelDataStructure.write();

				break;
			}
			case PacketType::RemoteChannelData: {
				if (!checkCRC(stream.getBuffer(), ac.settings.channelDataStructure.getRequiredBitCountForChannels()))
					return false;

				if (ac.settings.channelDataStructure.fields.empty()) {
					ESP_LOGE("PacketParser", "channel data structure is empty");

					return false;
				}

				uint8_t channelIndex = 0;

				for (auto field : ac.settings.channelDataStructure.fields) {
					for (uint8_t i = 0; i < field.count; ++i) {
						if (channelIndex >= ac.channels.instances.size()) {
							ESP_LOGE("PacketParser", "channel index %d >= channels count %d", channelIndex, ac.channels.instances.size());

							return false;
						}

						const auto channel = ac.channels.instances[channelIndex];

						switch (channel->getDataType()) {
							case ChannelDataType::unsignedInteger: {
								auto uintChannel = reinterpret_cast<UintChannel*>(channel);
								uintChannel->setValue(stream.readUint32(uintChannel->getBitDepth()));

								ESP_LOGI("PacketParser", "channel #%d, uint value: %d", channelIndex, uintChannel->getValue());

								break;
							}
							case ChannelDataType::boolean: {
								auto boolChannel = reinterpret_cast<BoolChannel*>(channel);
								boolChannel->setValue(stream.readBool());

								ESP_LOGI("PacketParser", "channel #%d, bool value: %d", channelIndex, boolChannel->getValue());

								break;
							}
						}

						channelIndex++;
					}
				}

				ac.channels.onValueUpdated();

				break;
			}
			case PacketType::RemoteMotorConfiguration: {
				const auto motorCount = readValueCountAndCheckCRC(
					stream,
					4,
					Motor::powerBitCount * 4 + 1
				);

				if (!motorCount)
					return false;

				ac.settings.motors.configurations.clear();
				ac.settings.motors.configurations.reserve(motorCount);

				MotorConfiguration configuration {};

				for (uint8_t i = 0; i < motorCount; ++i) {
					configuration.min = stream.readUint16(Motor::powerBitCount);
					configuration.max = stream.readUint16(Motor::powerBitCount);
					configuration.startup = stream.readUint16(Motor::powerBitCount);
					configuration.offset = stream.readInt16(Motor::powerBitCount);
					configuration.reverse = stream.readBool();
					configuration.sanitize();

					ac.settings.motors.configurations.push_back(configuration);

					ESP_LOGI("PacketParser", "motor index: %d, min: %d, max: %d, startup: %d, offset: %d, reverse: %d", i, configuration.min, configuration.max, configuration.startup, configuration.offset, configuration.reverse);
				}

				ac.motors.updateFromSettings();
				ac.settings.motors.write();

				// Updating motors position

				break;
			}
			default: {
				ESP_LOGE("PacketParser", "unknown packet type: %d", static_cast<uint8_t>(packetType));

				return false;
			}
		}

		return true;
	}
}