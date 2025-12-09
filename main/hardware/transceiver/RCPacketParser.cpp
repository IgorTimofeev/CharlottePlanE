#include "RCPacketParser.h"

#include "rc.h"
#include "hardware/motor.h"

namespace pizda {
	bool RCPacketParser::onParse(ReadableBitStream& stream, PacketType packetType) {
		auto& rc = RC::getInstance();

		switch (packetType) {
			case PacketType::RemoteChannelDataStructure: {
				const auto valueCount = stream.readUint8(8);

				ESP_LOGI("RCPacketParser", "data type count: %d", valueCount);

				rc.settings.channelDataStructure.fields.clear();
				rc.settings.channelDataStructure.fields.reserve(valueCount);

				ChannelDataStructureSettingsField field {};

				for (uint8_t i = 0; i < valueCount; ++i) {
					field.type = static_cast<ChannelDataType>(stream.readUint8(3));

					switch (field.type) {
						case ChannelDataType::unsignedInteger: {
							field.bitDepth = stream.readUint8(5);
							field.count = stream.readUint8(8);

							ESP_LOGI("RCPacketParser", "data type #%d, type: uint, bit depth: %d, count: %d", i, field.bitDepth, field.count);

							break;
						}
						case ChannelDataType::boolean: {
							field.bitDepth = 1;
							field.count = stream.readUint8(8);

							ESP_LOGI("RCPacketParser", "data type #%d, type: bool", i);

							break;
						}
						default: {
							ESP_LOGI("RCPacketParser", "unknown data type");
							return false;
						}
					}

					rc.settings.channelDataStructure.fields.push_back(field);
				}

				rc.channels.updateFromSettings();
				rc.settings.channelDataStructure.scheduleWrite();

				break;
			}
			case PacketType::RemoteChannelData: {
				if (!checkCRC(stream.getBuffer(), rc.settings.channelDataStructure.getRequiredBitCountForChannels()))
					return false;

				if (rc.settings.channelDataStructure.fields.empty()) {
					ESP_LOGE("RCPacketParser", "channel data structure is empty");

					return false;
				}

				uint8_t channelIndex = 0;

				for (auto field : rc.settings.channelDataStructure.fields) {
					for (uint8_t i = 0; i < field.count; ++i) {
						if (channelIndex >= rc.channels.instances.size()) {
							ESP_LOGE("RCPacketParser", "channel index %d >= channels count %d", channelIndex, rc.channels.instances.size());

							return false;
						}

						const auto channel = rc.channels.instances[channelIndex];

						switch (channel->getDataType()) {
							case ChannelDataType::unsignedInteger: {
								auto uintChannel = reinterpret_cast<UintChannel*>(channel);
								uintChannel->setValue(stream.readUint32(uintChannel->getBitDepth()));

								ESP_LOGI("RCPacketParser", "channel #%d, uint value: %d", channelIndex, uintChannel->getValue());

								break;
							}
							case ChannelDataType::boolean: {
								auto boolChannel = reinterpret_cast<BoolChannel*>(channel);
								boolChannel->setValue(stream.readBool());

								ESP_LOGI("RCPacketParser", "channel #%d, bool value: %d", channelIndex, boolChannel->getValue());

								break;
							}
						}

						channelIndex++;
					}
				}

				rc.channels.onValueUpdated();

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

				rc.settings.motors.configurations.clear();
				rc.settings.motors.configurations.reserve(motorCount);

				MotorConfiguration configuration {};

				for (uint8_t i = 0; i < motorCount; ++i) {
					configuration.min = stream.readUint16(Motor::powerBitCount);
					configuration.max = stream.readUint16(Motor::powerBitCount);
					configuration.startup = stream.readUint16(Motor::powerBitCount);
					configuration.offset = stream.readInt16(Motor::powerBitCount);
					configuration.reverse = stream.readBool();
					configuration.sanitize();

					rc.settings.motors.configurations.push_back(configuration);

					ESP_LOGI("RCPacketParser", "motor index: %d, min: %d, max: %d, startup: %d, offset: %d, reverse: %d", i, configuration.min, configuration.max, configuration.startup, configuration.offset, configuration.reverse);
				}

				rc.motors.updateFromSettings();
				rc.settings.motors.scheduleWrite();

				// Updating motors position

				break;
			}
			default: {
				ESP_LOGE("RCPacketParser", "unknown packet type: %d", static_cast<uint8_t>(packetType));

				return false;
			}
		}

		return true;
	}
}