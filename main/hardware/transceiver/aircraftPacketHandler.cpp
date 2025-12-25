#include "aircraftPacketHandler.h"

#include <utility>

#include <esp_timer.h>

#include "aircraft.h"
#include "hardware/motor.h"

namespace pizda {
	// -------------------------------- Reading --------------------------------
	
	bool AircraftPacketHandler::readPacket(BitStream& stream, PacketType packetType, uint8_t payloadLength) {
		switch (packetType) {
			case PacketType::remoteChannelDataStructure: {
				return readChannelDataStructurePacket(stream, payloadLength);
			}
			case PacketType::remoteChannelData: {
				return readChannelDataPacket(stream, payloadLength);
			}
			case PacketType::remoteMotorConfiguration: {
				return readMotorConfigurationPacket(stream, payloadLength);
			}
			case PacketType::remoteBaro: {
				return readRemoteBaroPacket(stream, payloadLength);
			}
			default: {
				ESP_LOGE(_logTag, "failed to read packet: unsupported type %d", std::to_underlying(packetType));
				return false;
			}
		}
	}

	bool AircraftPacketHandler::readChannelDataStructurePacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();

		const auto valueCount = stream.readUint8(8);

		ESP_LOGI(_logTag, "data type count: %d", valueCount);

		ac.settings.channelDataStructure.fields.clear();
		ac.settings.channelDataStructure.fields.reserve(valueCount);

		ChannelDataStructureSettingsField field {};

		for (uint8_t i = 0; i < valueCount; ++i) {
			field.type = static_cast<ChannelDataType>(stream.readUint8(3));

			switch (field.type) {
				case ChannelDataType::unsignedInteger: {
					field.bitDepth = stream.readUint8(5);
					field.count = stream.readUint8(8);

					ESP_LOGI(_logTag, "data type #%d, type: uint, bit depth: %d, count: %d", i, field.bitDepth, field.count);

					break;
				}
				case ChannelDataType::boolean: {
					field.bitDepth = 1;
					field.count = stream.readUint8(8);

					ESP_LOGI(_logTag, "data type #%d, type: bool, count: %d", i, field.count);

					break;
				}
				default: {
					ESP_LOGI(_logTag, "unknown data type");
					return false;
				}
			}

			ac.settings.channelDataStructure.fields.push_back(field);
		}

		ac.channels.updateFromDataStructure();
		ac.settings.channelDataStructure.write();

		return true;
	}

	bool AircraftPacketHandler::readChannelDataPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();

		if (!validatePayloadChecksumAndLength(
			stream,
			ac.settings.channelDataStructure.getRequiredBitCountForChannels(),
			payloadLength
		))
			return false;

		if (ac.settings.channelDataStructure.fields.empty()) {
			ESP_LOGE(_logTag, "channel data structure is empty");

			return false;
		}

		uint8_t channelIndex = 0;

		for (auto field : ac.settings.channelDataStructure.fields) {
			for (uint8_t i = 0; i < field.count; ++i) {
				const auto channel = ac.channels.getChannel(channelIndex);

				if (!channel)
					return false;

				switch (channel->getDataType()) {
					case ChannelDataType::unsignedInteger: {
						const auto uintChannel = reinterpret_cast<UintChannel*>(channel);
						uintChannel->setValue(stream.readUint32(uintChannel->getBitDepth()));

						ESP_LOGI(_logTag, "channel #%d, uint value: %d", channelIndex, uintChannel->getValue());

						break;
					}
					case ChannelDataType::boolean: {
						const auto boolChannel = reinterpret_cast<BoolChannel*>(channel);
						boolChannel->setValue(stream.readBool());

						ESP_LOGI(_logTag, "channel #%d, bool value: %d", channelIndex, boolChannel->getValue());

						break;
					}
				}

				channelIndex++;
			}
		}

		ac.updateHardwareFromChannels();

		return true;
	}

	bool AircraftPacketHandler::readMotorConfigurationPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		const auto motorCount = stream.readUint8(4);
		
		ESP_LOGI(_logTag, "motor count: %d", motorCount);

		if (!validatePayloadChecksumAndLength(
			stream,
			4 + (Motor::powerBitCount * 4 + 1) * motorCount,
			payloadLength
		))
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

			ESP_LOGI(_logTag, "motor index: %d, min: %d, max: %d, startup: %d, offset: %d, reverse: %d", i, configuration.min, configuration.max, configuration.startup, configuration.offset, configuration.reverse);
		}

		ac.motors.updateConfigurationsFromSettings();
		ac.settings.motors.write();

		return true;
	}
	
	bool AircraftPacketHandler::readRemoteBaroPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			sizeof(uint32_t) * 1 * 8,
			payloadLength
		))
			return false;
		
		ac.ahrs.setReferencePressurePa(sanitizeValue<uint32_t>(stream.readUint32(), 900'00, 1100'00));
		
		return true;
	}
	
	// -------------------------------- Writing --------------------------------
	
	bool AircraftPacketHandler::writePacket(BitStream& stream, PacketType packetType) {
		switch (packetType) {
			case PacketType::aircraftADIRS:
				return writeAircraftADIRSPacket(stream);
			
			default:
				ESP_LOGE(_logTag, "failed to write packet: unsupported type %d", std::to_underlying(packetType));
				return false;
		}
	}
	
	bool AircraftPacketHandler::writeAircraftADIRSPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		// Payload
		auto writeEbanina = [&stream](float value, uint8_t bits) {
			const auto uintValue = static_cast<uint16_t>((value / (2.f * std::numbers::pi_v<float>) + 0.5f) * (1 << bits));
			
			stream.writeUint16(uintValue, bits);
		};
		
		writeEbanina(ac.ahrs.getRollRad(), 12);
		writeEbanina(ac.ahrs.getPitchRad(), 12);
		writeEbanina(ac.ahrs.getYawRad(), 12);
		
		stream.writeUint8(static_cast<uint8_t>(ac.ahrs.getAccelVelocityMs()), 8);
		stream.writeInt16(static_cast<int16_t>(ac.ahrs.getAltitudeM()), 16);
		
		return true;
	}
	
	void AircraftPacketHandler::onConnectionStateChanged(TransceiverConnectionState fromState, TransceiverConnectionState toState) {
		auto& ac = Aircraft::getInstance();
		
		switch (toState) {
			case TransceiverConnectionState::connected: {
				ac.lights.setEmergencyEnabled(true);
				
				break;
			}
			case TransceiverConnectionState::disconnected: {
				ac.lights.setEmergencyEnabled(true);
				
				break;
			}
			default: break;
		}
	}
}