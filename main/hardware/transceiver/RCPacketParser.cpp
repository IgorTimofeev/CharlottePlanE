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

				rc.settings.remoteChannelDataStructure.fields.clear();
				rc.settings.remoteChannelDataStructure.fields.reserve(valueCount);

				RemoteChannelDataStructureSettingsField field {};

				for (uint8_t i = 0; i < valueCount; ++i) {
					field.type = static_cast<RemoteChannelDataStructureSettingsChannelType>(stream.readUint8(3));

					switch (field.type) {
						case RemoteChannelDataStructureSettingsChannelType::unsignedInteger: {
							field.bitDepth = stream.readUint8(5);
							field.count = stream.readUint8(8);

							ESP_LOGI("RCPacketParser", "data type num: %d, type: uint, bit depth: %d, count: %d", i, field.bitDepth, field.count);

							break;
						}
						case RemoteChannelDataStructureSettingsChannelType::boolean: {
							field.bitDepth = 1;
							field.count = stream.readUint8(8);

							ESP_LOGI("RCPacketParser", "data type num: %d, type: bool", i);

							break;
						}
						default: {
							ESP_LOGI("RCPacketParser", "unknown data type");
							return false;
						}
					}

					rc.settings.remoteChannelDataStructure.fields.push_back(field);
				}

				rc.settings.remoteChannelDataStructure.scheduleWrite();

				break;
			}
			case PacketType::RemoteChannelData: {
				if (!checkCRC(stream.getBuffer(), rc.settings.remoteChannelDataStructure.getRequiredBitCountForChannels()))
					return false;

				if (rc.settings.remoteChannelDataStructure.fields.empty()) {
					ESP_LOGE("RCPacketParser", "channel data structure is empty");

					return false;
				}

				size_t channelIndex = 0;

				for (auto field : rc.settings.remoteChannelDataStructure.fields) {
					for (uint8_t i = 0; i < field.count; ++i) {
						switch (field.type) {
							case RemoteChannelDataStructureSettingsChannelType::unsignedInteger: {
								const auto value = stream.readUint32(field.bitDepth);
								rc.channels.setValue(channelIndex, value);

								ESP_LOGI("RCPacketParser", "channel: %d, uint value: %d", channelIndex, value);

								break;
							}
							case RemoteChannelDataStructureSettingsChannelType::boolean: {
								const auto value = stream.readBool();
								rc.channels.setValue(channelIndex, value);

								ESP_LOGI("RCPacketParser", "channel: %d, bool value: %d", channelIndex, value);

								break;
							}
						}

						channelIndex++;

						if (channelIndex >= 255) {
							ESP_LOGE("RCPacketParser", "channel index %d is out of range", channelIndex);

							return false;
						}
					}
				}

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

				MotorConfiguration settings {};

				for (uint8_t i = 0; i < motorCount; ++i) {
					settings.min = stream.readUint16(Motor::powerBitCount);
					settings.max = stream.readUint16(Motor::powerBitCount);
					settings.startup = stream.readUint16(Motor::powerBitCount);
					settings.offset = stream.readInt16(Motor::powerBitCount);
					settings.reverse = stream.readBool();

					rc.settings.motors.configurations.push_back(settings);

					ESP_LOGI("RCPacketParser", "motor index: %d, min: %d, max: %d, startup: %d, offset: %d, reverse: %d", i, settings.min, settings.max, settings.startup, settings.offset, settings.reverse);
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