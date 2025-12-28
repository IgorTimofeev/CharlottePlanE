#include "aircraftPacketHandler.h"

#include <utility>

#include <esp_timer.h>

#include "aircraft.h"
#include "hardware/motor.h"

namespace pizda {

	// -------------------------------- Reading --------------------------------
	
	bool AircraftPacketHandler::readPacket(BitStream& stream, PacketType packetType, uint8_t payloadLength) {
		switch (packetType) {
			case PacketType::remoteChannelsDataStructure: {
				return readRemoteChannelDataStructurePacket(stream, payloadLength);
			}
			case PacketType::remoteChannelsData: {
				return readRemoteChannelsDataPacket(stream, payloadLength);
			}
			case PacketType::remoteMotorConfiguration: {
				return readMotorConfigurationPacket(stream, payloadLength);
			}
			case PacketType::remoteAuxiliary: {
				return readRemoteAuxiliary0Packet(stream, payloadLength);
			}
			default: {
				ESP_LOGE(_logTag, "failed to read packet: unsupported type %d", std::to_underlying(packetType));
				return false;
			}
		}
	}

	bool AircraftPacketHandler::readRemoteChannelDataStructurePacket(BitStream& stream, uint8_t payloadLength) {
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

	bool AircraftPacketHandler::readRemoteChannelsDataPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteChannelsPacket::motorLengthBits * 6 + 4,
			payloadLength
		))
			return false;
		
		const auto readChannel = [&stream]() {
			return stream.readUint16(RemoteChannelsPacket::motorLengthBits) * Motor::powerMaxValue / ((1 << RemoteChannelsPacket::motorLengthBits) - 1);
		};
		
		ac.channels.getUintChannel(ChannelType::throttle)->setValue(readChannel());
		ac.channels.getUintChannel(ChannelType::ailerons)->setValue(readChannel());
		ac.channels.getUintChannel(ChannelType::elevator)->setValue(readChannel());
		ac.channels.getUintChannel(ChannelType::rudder)->setValue(readChannel());
		ac.channels.getUintChannel(ChannelType::flaps)->setValue(readChannel());
		ac.channels.getUintChannel(ChannelType::unused)->setValue(readChannel());
		
		// Lights
		ac.channels.getBoolChannel(ChannelType::navLights)->setValue(stream.readBool());
		ac.channels.getBoolChannel(ChannelType::strobeLights)->setValue(stream.readBool());
		ac.channels.getBoolChannel(ChannelType::landingLights)->setValue(stream.readBool());
		ac.channels.getBoolChannel(ChannelType::cabinLights)->setValue(stream.readBool());
		
		ac.updateHardwareFromChannels();
		
		return true;
		
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
			4 + (12 * 4 + 1) * motorCount,
			payloadLength
		))
			return false;
		
		ac.settings.motors.configurations.clear();
		ac.settings.motors.configurations.reserve(motorCount);

		MotorConfiguration configuration {};

		for (uint8_t i = 0; i < motorCount; ++i) {
			configuration.min = stream.readUint16(12);
			configuration.max = stream.readUint16(12);
			configuration.startup = stream.readUint16(12);
			configuration.offset = stream.readInt16(12);
			configuration.reverse = stream.readBool();
			configuration.sanitize();

			ac.settings.motors.configurations.push_back(configuration);

			ESP_LOGI(_logTag, "motor index: %d, min: %d, max: %d, startup: %d, offset: %d, reverse: %d", i, configuration.min, configuration.max, configuration.startup, configuration.offset, configuration.reverse);
		}

		ac.motors.updateConfigurationsFromSettings();
		ac.settings.motors.write();

		return true;
	}
	
	bool AircraftPacketHandler::readRemoteAuxiliary0Packet(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteAuxiliary0Packet::referencePressureLengthBits + 3,
			payloadLength
		))
			return false;
		
		// Reference pressure
		const auto referencePressureDaPa = stream.readUint16(RemoteAuxiliary0Packet::referencePressureLengthBits);
		
		ac.ahrs.setReferencePressurePa(sanitizeValue<uint32_t>(static_cast<uint32_t>(referencePressureDaPa) * 10, 900'00, 1100'00));
		
		return true;
	}
	
	// -------------------------------- Writing --------------------------------
	
	bool AircraftPacketHandler::writePacket(BitStream& stream, PacketType packetType) {
		switch (packetType) {
			case PacketType::aircraftADIRS:
				return writeAircraftADIRSPacket(stream);
			
			case PacketType::aircraftStatistics:
				return writeAircraftStatisticsPacket(stream);
			
			case PacketType::aircraftAutopilot:
				return writeAircraftAutopilotPacket(stream);
			
			default:
				ESP_LOGE(_logTag, "failed to write packet: unsupported type %d", std::to_underlying(packetType));
				return false;
		}
	}
	
	bool AircraftPacketHandler::writeAircraftADIRSPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		// Roll / pitch / yaw
		auto writeRadians = [&stream](float value, uint8_t bits) {
			const auto uintValue = static_cast<uint16_t>((value / (2.f * std::numbers::pi_v<float>) + 0.5f) * ((1 << bits) - 1));
			
			stream.writeUint16(uintValue, bits);
		};
		
		// Roll range is [-180; 180] deg
		writeRadians(ac.ahrs.getRollRad(), AircraftADIRSPacket::rollLengthBits);
		// Pitch is [-90; 90] deg, but we can't use fewer bits
		writeRadians(ac.ahrs.getPitchRad(), AircraftADIRSPacket::pitchLengthBits);
		// Same as roll, [-180; 180] deg
		writeRadians(ac.ahrs.getYawRad(), AircraftADIRSPacket::yawLengthBits);

		// Slip & skid
		const auto slipAndSkidValue = static_cast<uint8_t>(static_cast<float>((1 << AircraftADIRSPacket::slipAndSkidLengthBits) - 1) * (ac.ahrs.getSlipAndSkidFactor() + 1.f) / 2.f);
		
		stream.writeUint8(slipAndSkidValue, AircraftADIRSPacket::slipAndSkidLengthBits);
		
		// Speed
		stream.writeUint8(static_cast<uint8_t>(ac.ahrs.getAccelVelocityMs()), AircraftADIRSPacket::speedLengthBits);
		
		// Altitude
		const auto altitudeClamped = std::clamp<float>(ac.ahrs.getAltitudeM(), AircraftADIRSPacket::altitudeMin, AircraftADIRSPacket::altitudeMax);
		const auto altitudeFactor = (altitudeClamped - static_cast<float>(AircraftADIRSPacket::altitudeMin)) / static_cast<float>(AircraftADIRSPacket::altitudeMax - AircraftADIRSPacket::altitudeMin);
		const auto altitudeUint16 = static_cast<uint16_t>(altitudeFactor * static_cast<float>((1 << AircraftADIRSPacket::altitudeLengthBits) - 1));
		
		stream.writeInt16(altitudeUint16, AircraftADIRSPacket::altitudeLengthBits);

//		stream.writeUint16(2048, 12);
//		stream.writeUint16(2048, 12);
//		stream.writeUint16(2048, 12);
//
//		stream.writeUint8(125, 8);
//		stream.writeInt16(100, 16);
		
		return true;
	}
	
	bool AircraftPacketHandler::writeAircraftStatisticsPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		// Throttle
		stream.writeUint8(
			ac.motors.getMotor(MotorType::throttle)->getPower() * ((1 << AircraftStatisticsPacket::throttleLengthBits) - 1) / Motor::powerMaxValue,
			AircraftStatisticsPacket::throttleLengthBits
		);
		
		// 60.014002019765776, 29.717151511256816
		// ОПЯТЬ ЖЕНЩИНЫ??? ФЕДЯ СУКА ЭТО ТЫ ЕБЛАН СДЕЛАЛ
		
		// Lat
		const auto latRad = toRadians(60.014002019765776f);
		// Mapping from [-90; 90] to [0; 180] and then to [0; 1]
		const auto latFactor = (latRad + std::numbers::pi_v<float> / 2.f) / std::numbers::pi_v<float>;
		const auto latValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftStatisticsPacket::latLengthBits) - 1) * latFactor);
		
		stream.writeUint32(latValue, AircraftStatisticsPacket::latLengthBits);
		
		// Lon
		const auto lonRad = toRadians(29.717151511256816);
		// Mapping from [0; 360] to [0; 1]
		const auto lonFactor = lonRad / (2 * std::numbers::pi_v<float>);
		const auto lonValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftStatisticsPacket::lonLengthBits) - 1) * lonFactor);
		
		stream.writeUint32(lonValue, AircraftStatisticsPacket::lonLengthBits);
		
		// Battery, daV
		stream.writeUint16(125, AircraftStatisticsPacket::batteryLengthBits);
		
		return true;
	}
	
	bool AircraftPacketHandler::writeAircraftAutopilotPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		return true;
	}
	
	void AircraftPacketHandler::onConnectionStateChanged(TransceiverConnectionState fromState, TransceiverConnectionState toState) {
		auto& ac = Aircraft::getInstance();
		
		switch (toState) {
			case TransceiverConnectionState::connected: {
				ac.lights.setEmergencyEnabled(false);
				
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