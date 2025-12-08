#include "RCPacketParser.h"

#include "rc.h"
#include "hardware/motor.h"

namespace pizda {
	bool RCPacketParser::onParse(ReadableBitStream& stream, PacketType packetType) {
		auto& rc = RC::getInstance();

		switch (packetType) {
			case PacketType::RemoteChannelDataStructure: {
				const auto dataTypeCount = stream.readUint8(3);

				ESP_LOGI("RCPacketParser", "data type count: %d", dataTypeCount);

				rc.settings.remoteChannelDataStructure.fields.clear();

				RemoteChannelDataStructureSettingsField channel {};

				for (uint8_t i = 0; i < dataTypeCount; ++i) {
					channel.type = static_cast<RemoteChannelDataStructureSettingsChannelType>(stream.readUint8(3));

					switch (channel.type) {
						case RemoteChannelDataStructureSettingsChannelType::Int: {
							channel.bitDepth = stream.readUint8(5);

							ESP_LOGI("RCPacketParser", "data type %d: int", i);
							ESP_LOGI("RCPacketParser", "data type %d bit depth: %d", i, channel.bitDepth);

							break;
						}
						case RemoteChannelDataStructureSettingsChannelType::Uint: {
							channel.bitDepth = stream.readUint8(5);

							ESP_LOGI("RCPacketParser", "data type %d: uint", i);
							ESP_LOGI("RCPacketParser", "data type %d bit depth: %d", i, channel.bitDepth);

							break;
						}
						case RemoteChannelDataStructureSettingsChannelType::Bool: {
							channel.bitDepth = 1;

							ESP_LOGI("RCPacketParser", "data type %d: bool", i);

							break;
						}
						default: {
							ESP_LOGI("RCPacketParser", "unknown data type");
							continue;
						}
					}

					channel.count = stream.readUint8(8);
					ESP_LOGI("RCPacketParser", "data type %d count: %d", i, channel.count);

					rc.settings.remoteChannelDataStructure.fields.push_back(channel);
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

				uint8_t channelIndex = 0;

				for (auto field : rc.settings.remoteChannelDataStructure.fields) {
					for (uint8_t i = 0; i < field.count; ++i) {
						onChannelRead(stream, field, channelIndex);

						channelIndex++;
					}
				}

				break;
			}
			case PacketType::RemoteMotorConfiguration: {
				const auto motorCount = readValueCountAndCheckCRC(
					stream,
					8,
					4,
					Motor::powerBitCount + Motor::powerBitCount + Motor::powerBitCount + 1
				);

				if (!motorCount)
					return false;

				for (uint8_t motorIndex = 0; motorIndex < motorCount; ++motorIndex)
					onMotorConfiguration(stream, motorIndex);

				break;
			}
			default: {
				ESP_LOGE("RCPacketParser", "unknown packet type: %d", static_cast<uint8_t>(packetType));

				return false;
			}
		}

		return true;
	}

	void RCPacketParser::onChannelRead(ReadableBitStream& stream, const RemoteChannelDataStructureSettingsField& field, uint8_t channelIndex) {
		auto& rc = RC::getInstance();

		ESP_LOGI("RCPacketParser", "channel index: %d", channelIndex);

		switch (channelIndex) {
			// Uint12
			case 0: {
				rc.motors.setThrottle(stream.readUint16(field.bitDepth));
				ESP_LOGI("RCPacketParser", "throttle: %d", rc.motors.getThrottle());

				break;
			}
			case 1: {
				rc.motors.setReverseThrottle(stream.readUint16(field.bitDepth));
				ESP_LOGI("RCPacketParser", "reverse: %d", rc.motors.getThrottle());

				break;
			}
			case 2: {
				rc.motors.setAilerons(stream.readUint16(field.bitDepth));
				ESP_LOGI("RCPacketParser", "ailerons: %d", rc.motors.getAilerons());

				break;
			}
			case 3: {
				rc.motors.setElevator(stream.readUint16(field.bitDepth));
				ESP_LOGI("RCPacketParser", "elevator: %d", rc.motors.getElevator());

				break;
			}
			case 4: {
				rc.motors.setRudder(stream.readUint16(field.bitDepth));
				ESP_LOGI("RCPacketParser", "rudder: %d", rc.motors.getRudder());

				break;
			}
			case 5: {
				rc.motors.setFlaps(stream.readUint16(field.bitDepth));
				ESP_LOGI("RCPacketParser", "flaps: %d", rc.motors.getFlaps());

				break;
			}

			// Bool
			case 6: {
				rc.lights.setNavigationEnabled(stream.readBool());
				ESP_LOGI("RCPacketParser", "navigation: %d", rc.lights.isNavigationEnabled());

				break;
			}
			case 7: {
				rc.lights.setStrobeEnabled(stream.readBool());
				ESP_LOGI("RCPacketParser", "strobe: %d", rc.lights.isStrobeEnabled());

				break;
			}
			case 8: {
				rc.lights.setLandingEnabled(stream.readBool());
				ESP_LOGI("RCPacketParser", "landing: %d", rc.lights.isLandingEnabled());

				break;
			}
			case 9: {
				rc.lights.setCabinEnabled(stream.readBool());
				ESP_LOGI("RCPacketParser", "cabin: %d", rc.lights.isCabinEnabled());

				break;
			}
			default: {
				ESP_LOGI("RCPacketParser", "channel index out of range: %d", channelIndex);

				break;
			}
		}
	}

	void RCPacketParser::onMotorConfiguration(ReadableBitStream& stream, uint8_t motorIndex) {
		auto& rc = RC::getInstance();

		const auto readIntoSettings = [&stream](MotorSettings& settings) {
			settings.min = stream.readUint16(Motor::powerBitCount);
			settings.max = stream.readUint16(Motor::powerBitCount);
			settings.offset = stream.readInt16(Motor::powerBitCount);
			settings.reverse = stream.readBool();
		};

		switch (motorIndex) {
			case 0: {
				readIntoSettings(rc.settings.motors.leftThrottle);
				ESP_LOGI("RCPacketParser", "left throttle min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftThrottle.min, rc.settings.motors.leftThrottle.max, rc.settings.motors.leftThrottle.offset, rc.settings.motors.leftThrottle.reverse);

				break;
			}
			case 1: {
				readIntoSettings(rc.settings.motors.rightThrottle);
				ESP_LOGI("RCPacketParser", "right throttle min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightThrottle.min, rc.settings.motors.rightThrottle.max, rc.settings.motors.rightThrottle.offset, rc.settings.motors.rightThrottle.reverse);

				break;
			}
			case 2: {
				readIntoSettings(rc.settings.motors.leftAileron);
				ESP_LOGI("RCPacketParser", "left aileron min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftAileron.min, rc.settings.motors.leftAileron.max, rc.settings.motors.leftAileron.offset, rc.settings.motors.leftAileron.reverse);

				break;
			}
			case 3: {
				readIntoSettings(rc.settings.motors.rightAileron);
				ESP_LOGI("RCPacketParser", "right aileron min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightAileron.min, rc.settings.motors.rightAileron.max, rc.settings.motors.rightAileron.offset, rc.settings.motors.rightAileron.reverse);

				break;
			}
			case 4: {
				readIntoSettings(rc.settings.motors.leftTail);
				ESP_LOGI("RCPacketParser", "left tail min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftTail.min, rc.settings.motors.leftTail.max, rc.settings.motors.leftTail.offset, rc.settings.motors.leftTail.reverse);

				break;
			}
			case 5: {
				readIntoSettings(rc.settings.motors.rightTail);
				ESP_LOGI("RCPacketParser", "right tail min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightTail.min, rc.settings.motors.rightTail.max, rc.settings.motors.rightTail.offset, rc.settings.motors.rightTail.reverse);

				break;
			}
			case 6: {
				readIntoSettings(rc.settings.motors.leftFlap);
				ESP_LOGI("RCPacketParser", "left flap min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftFlap.min, rc.settings.motors.leftFlap.max, rc.settings.motors.leftFlap.offset, rc.settings.motors.leftFlap.reverse);

				break;
			}
			case 7: {
				readIntoSettings(rc.settings.motors.rightFlap);
				ESP_LOGI("RCPacketParser", "right flap min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightFlap.min, rc.settings.motors.rightFlap.max, rc.settings.motors.rightFlap.offset, rc.settings.motors.rightFlap.reverse);
				break;
			}
			default: {
				ESP_LOGI("RCPacketParser", "unsupported motor calibration index: %d", motorIndex);
				return;
			}
		}

		rc.settings.motors.scheduleWrite();

		// Updating motors position
		rc.motors.setAilerons(rc.motors.getAilerons());
		rc.motors.setFlaps(rc.motors.getFlaps());
	}
}