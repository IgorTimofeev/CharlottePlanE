#include "aircraftPacketHandler.h"

#include <utility>

#include <esp_timer.h>

#include "aircraft.h"
#include "hardware/motor.h"

namespace pizda {
	// -------------------------------- PacketSequenceItem --------------------------------
	
	PacketSequenceItem::PacketSequenceItem(AircraftPacketType type, uint8_t count, bool useEnqueued) : _type(type), _count(count), _useEnqueued(useEnqueued) {
	
	}
	
	AircraftPacketType PacketSequenceItem::getType() const {
		return _type;
	}
	
	uint8_t PacketSequenceItem::getCount() const {
		return _count;
	}
	
	bool PacketSequenceItem::useEnqueued() const {
		return _useEnqueued;
	}
	
	// -------------------------------- Generic --------------------------------
	
	void AircraftPacketHandler::onStart() {
		ESP_LOGI(_logTag, "started");
		
		while (true) {
			if (receive(1'000'000)) {
				vTaskDelay(pdMS_TO_TICKS(20));
				transmit(1'000'000);
			}
		}
	}
	
	void AircraftPacketHandler::onIsConnectedChanged() {
		auto& ac = Aircraft::getInstance();
		
		if (isConnected()) {
			ac.lights.setEmergencyEnabled(false);
		}
		else {
			ac.lights.setEmergencyEnabled(true);
		}
	}
	
	// -------------------------------- Receiving --------------------------------
	
	bool AircraftPacketHandler::onReceive(BitStream& stream, RemotePacketType packetType, uint8_t payloadLength) {
		switch (packetType) {
			case RemotePacketType::NOP:
				return receiveNOPPacket(stream, payloadLength);
				
			case RemotePacketType::channelsDataStructure:
				return receiveRemoteChannelDataStructurePacket(stream, payloadLength);
				
			case RemotePacketType::channelsData:
				return receiveRemoteChannelsDataPacket(stream, payloadLength);
				
			case RemotePacketType::motorConfiguration:
				return receiveMotorConfigurationPacket(stream, payloadLength);
			
			case RemotePacketType::auxiliary:
				return receiveRemoteAuxiliaryPacket(stream, payloadLength);
			
			default:
				ESP_LOGE(_logTag, "failed to receive packet: unsupported type %d", packetType);
				return false;
		}
	}
	
	bool AircraftPacketHandler::receiveNOPPacket(BitStream& stream, uint8_t payloadLength) {
		return true;
	}

	bool AircraftPacketHandler::receiveRemoteChannelDataStructurePacket(BitStream& stream, uint8_t payloadLength) {
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

	bool AircraftPacketHandler::receiveRemoteChannelsDataPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteChannelsPacket::motorLengthBits * 6 + 4,
			payloadLength
		))
			return false;
		
		const auto receiveChannel = [&stream]() {
			return stream.readUint16(RemoteChannelsPacket::motorLengthBits) * Motor::powerMaxValue / ((1 << RemoteChannelsPacket::motorLengthBits) - 1);
		};
		
		ac.channels.getUintChannel(ChannelType::throttle)->setValue(receiveChannel());
		ac.channels.getUintChannel(ChannelType::ailerons)->setValue(receiveChannel());
		ac.channels.getUintChannel(ChannelType::elevator)->setValue(receiveChannel());
		ac.channels.getUintChannel(ChannelType::rudder)->setValue(receiveChannel());
		ac.channels.getUintChannel(ChannelType::flaps)->setValue(receiveChannel());
		ac.channels.getUintChannel(ChannelType::unused)->setValue(receiveChannel());
		
		// Lights
		ac.channels.getBoolChannel(ChannelType::navLights)->setValue(stream.readBool());
		ac.channels.getBoolChannel(ChannelType::strobeLights)->setValue(stream.readBool());
		ac.channels.getBoolChannel(ChannelType::landingLights)->setValue(stream.readBool());
		ac.channels.getBoolChannel(ChannelType::cabinLights)->setValue(stream.readBool());
		
		ac.fbw.applyData();
		
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
		
		ac.fbw.applyData();

		return true;
	}

	bool AircraftPacketHandler::receiveMotorConfigurationPacket(BitStream& stream, uint8_t payloadLength) {
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
	
	bool AircraftPacketHandler::receiveRemoteAuxiliaryPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteAuxiliaryPacket::referencePressureLengthBits
				+ RemoteAuxiliaryPacket::autopilotSpeedLengthBits + 1
				+ RemoteAuxiliaryPacket::autopilotHeadingLengthBits + 1
				+ RemoteAuxiliaryPacket::autopilotAltitudeLengthBits + 1,
			payloadLength
		))
			return false;
		
		// Reference pressure
		const auto referencePressureDaPa = stream.readUint16(RemoteAuxiliaryPacket::referencePressureLengthBits);
		
		ac.adirs.setReferencePressurePa(sanitizeValue<uint32_t>(static_cast<uint32_t>(referencePressureDaPa) * 10, 900'00, 1100'00));
		
		// -------------------------------- Autopilot --------------------------------
		
		// Speed
		const auto speedFactor =
			static_cast<float>(stream.readUint8(RemoteAuxiliaryPacket::autopilotSpeedLengthBits))
			/ static_cast<float>((1 << RemoteAuxiliaryPacket::autopilotSpeedLengthBits) - 1);
		
		ac.remoteData.raw.autopilot.speedMPS = static_cast<float>(RemoteAuxiliaryPacket::autopilotSpeedMax) * speedFactor;
		ac.remoteData.raw.autopilot.autoThrottle = stream.readBool();
		
		// Heading
		ac.remoteData.raw.autopilot.headingDeg = stream.readUint16(RemoteAuxiliaryPacket::autopilotHeadingLengthBits);
		ac.remoteData.raw.autopilot.headingHold = stream.readBool();
		
		// Altitude
		const auto altitudeFactor =
			static_cast<float>(stream.readUint16(RemoteAuxiliaryPacket::autopilotAltitudeLengthBits))
			/ static_cast<float>((1 << RemoteAuxiliaryPacket::autopilotAltitudeLengthBits) - 1);
		
		ac.remoteData.raw.autopilot.altitudeM =
			RemoteAuxiliaryPacket::autopilotAltitudeMin
			+ (RemoteAuxiliaryPacket::autopilotAltitudeMax - RemoteAuxiliaryPacket::autopilotAltitudeMin) * altitudeFactor;
		
		ac.remoteData.raw.autopilot.levelChange = stream.readBool();
		
		return true;
	}
	
	// -------------------------------- Transmitting --------------------------------
	
	AircraftPacketType AircraftPacketHandler::getTransmitPacketType() {
		switch (getRemoteState()) {
			default: {
				const auto& item = _packetSequence[_packetSequenceIndex];
				
				const auto next = [this, &item]() {
					_packetSequenceItemCounter++;
					
					if (_packetSequenceItemCounter < item.getCount())
						return;
					
					_packetSequenceItemCounter = 0;
					
					_packetSequenceIndex++;
					
					if (_packetSequenceIndex >= _packetSequence.size())
						_packetSequenceIndex = 0;
				};
				
				// Enqueued
				if (item.useEnqueued() && !_packetQueue.empty()) {
					const auto packetType = _packetQueue.front();
					_packetQueue.pop();
					
					next();
					
					return packetType;
				}
					// Normal
				else {
					const auto packetType = item.getType();
					
					next();
					
					return packetType;
				}
			}
		}
	}
	
	bool AircraftPacketHandler::onTransmit(BitStream& stream, AircraftPacketType packetType) {
		switch (packetType) {
			case AircraftPacketType::ADIRS:
				return transmitAircraftADIRSPacket(stream);
			
			case AircraftPacketType::auxiliary:
				return transmitAircraftAuxiliaryPacket(stream);
			
			default:
				ESP_LOGE(_logTag, "failed to write packet: unsupported type %d", packetType);
				
				return false;
		}
	}
	
	bool AircraftPacketHandler::transmitAircraftADIRSPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		// Roll / pitch / yaw
		writeRadians(stream, ac.adirs.getRollRad(), 2.f * std::numbers::pi_v<float>, AircraftADIRSPacket::rollLengthBits);
		writeRadians(stream, ac.adirs.getPitchRad(), std::numbers::pi_v<float>, AircraftADIRSPacket::pitchLengthBits);
		writeRadians(stream, ac.adirs.getYawRad(), 2.f * std::numbers::pi_v<float>, AircraftADIRSPacket::yawLengthBits);

		// Slip & skid
		const auto slipAndSkidValue = static_cast<uint8_t>(
			static_cast<float>((1 << AircraftADIRSPacket::slipAndSkidLengthBits) - 1)
			// Mapping from [-1.0; 1.0] to [0.0; 1.0]
			* (ac.adirs.getSlipAndSkidFactor() + 1.f) / 2.f
		);
		
		stream.writeUint8(slipAndSkidValue, AircraftADIRSPacket::slipAndSkidLengthBits);
		
		// Speed
		const auto speedFactor =
			std::min<float>(ac.adirs.getAccelVelocityMPS(), AircraftADIRSPacket::speedMax)
		    / static_cast<float>(AircraftADIRSPacket::speedMax);
		
		const auto speedMapped = static_cast<float>((1 << AircraftADIRSPacket::speedLengthBits) - 1) * speedFactor;
		
		stream.writeUint8(static_cast<uint8_t>(speedMapped), AircraftADIRSPacket::speedLengthBits);
		
		// Altitude
		const auto altitudeClamped = std::clamp<float>(ac.adirs.getAltitudeM(), AircraftADIRSPacket::altitudeMin, AircraftADIRSPacket::altitudeMax);
		
		const auto altitudeFactor =
			(altitudeClamped - static_cast<float>(AircraftADIRSPacket::altitudeMin))
			/ static_cast<float>(AircraftADIRSPacket::altitudeMax - AircraftADIRSPacket::altitudeMin);
		
		const auto altitudeUint16 = static_cast<uint16_t>(altitudeFactor * static_cast<float>((1 << AircraftADIRSPacket::altitudeLengthBits) - 1));
		
		stream.writeUint16(altitudeUint16, AircraftADIRSPacket::altitudeLengthBits);
		
		// Throttle
		const auto motor = ac.motors.getMotor(MotorType::throttle);
		
		stream.writeUint8(
			(motor ? motor->getPower() : 0) * ((1 << AircraftADIRSPacket::throttleLengthBits) - 1) / Motor::powerMaxValue,
			AircraftADIRSPacket::throttleLengthBits
		);
		
		// Roll
		writeRadians(stream, ac.fbw.getTargetRollRad(), AircraftADIRSPacket::autopilotRollRangeRad, AircraftADIRSPacket::autopilotRollLengthBits);
		
		// Pitch
		writeRadians(stream, ac.fbw.getTargetPitchRad(), AircraftADIRSPacket::autopilotPitchRangeRad, AircraftADIRSPacket::autopilotPitchLengthBits);
		
		return true;
	}
	
	bool AircraftPacketHandler::transmitAircraftAuxiliaryPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		// 60.014002019765776, 29.717151511256816
		// ОПЯТЬ ЖЕНЩИНЫ??? ФЕДЯ СУКА ЭТО ТЫ ЕБЛАН СДЕЛАЛ
		
		// Lat
		const auto latRad = toRadians(60.014002019765776f);
		// Mapping from [-90; 90] to [0; 180] and then to [0; 1]
		const auto latFactor = (latRad + std::numbers::pi_v<float> / 2.f) / std::numbers::pi_v<float>;
		const auto latValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftAuxiliaryPacket::latLengthBits) - 1) * latFactor);
		
		stream.writeUint32(latValue, AircraftAuxiliaryPacket::latLengthBits);
		
		// Lon
		const auto lonRad = toRadians(29.717151511256816);
		// Mapping from [0; 360] to [0; 1]
		const auto lonFactor = lonRad / (2 * std::numbers::pi_v<float>);
		const auto lonValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftAuxiliaryPacket::lonLengthBits) - 1) * lonFactor);
		
		stream.writeUint32(lonValue, AircraftAuxiliaryPacket::lonLengthBits);
		
		// Battery, daV
		stream.writeUint16(125, AircraftAuxiliaryPacket::batteryLengthBits);
		
		return true;
	}
	
	void AircraftPacketHandler::writeRadians(BitStream& stream, float value, float range, uint8_t bits) {
		const auto uintValue = static_cast<uint16_t>((value / range + 0.5f) * ((1 << bits) - 1));
		
		stream.writeUint16(uintValue, bits);
	}
}