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
				
			case RemotePacketType::controls:
				return receiveRemoteControlsPacket(stream, payloadLength);
			
			case RemotePacketType::trim:
				return receiveRemoteTrimPacket(stream, payloadLength);
				
			case RemotePacketType::lights:
				return receiveRemoteLightsPacket(stream, payloadLength);
			
			case RemotePacketType::baro:
				return receiveRemoteBaroPacket(stream, payloadLength);
			
			case RemotePacketType::autopilot:
				return receiveRemoteAutopilotPacket(stream, payloadLength);
				
			case RemotePacketType::motorConfiguration:
				return receiveMotorConfigurationPacket(stream, payloadLength);
			
			default:
				ESP_LOGE(_logTag, "failed to receive packet: unsupported type %d", packetType);
				return false;
		}
	}
	
	bool AircraftPacketHandler::receiveNOPPacket(BitStream& stream, uint8_t payloadLength) {
		return true;
	}

	bool AircraftPacketHandler::receiveRemoteControlsPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteControlsPacket::motorLengthBits * 6,
			payloadLength
		))
			return false;
		
		const auto readMotor = [&stream]() {
			return
			    static_cast<float>(stream.readUint16(RemoteControlsPacket::motorLengthBits))
				/ static_cast<float>((1 << RemoteControlsPacket::motorLengthBits) - 1);
		};
		
		ac.remoteData.raw.controls.throttle = readMotor();
		ac.remoteData.raw.controls.ailerons = readMotor();
		ac.remoteData.raw.controls.elevator = readMotor();
		ac.remoteData.raw.controls.rudder = readMotor();
		ac.remoteData.raw.controls.flaps = readMotor();
		ac.remoteData.raw.controls.unused = readMotor();
		
		ac.fbw.applyData();
		
		return true;
	}
	
	bool AircraftPacketHandler::receiveRemoteTrimPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteTrimPacket::valueLengthBits * 3,
			payloadLength
		))
			return false;
		
		const auto read = [&stream]() {
			return
				// Mapping to motorPowerMaxValue / 2
			    static_cast<int32_t>(stream.readInt16(RemoteTrimPacket::valueLengthBits))
				* (Motor::powerMaxValue / 2)
				/ ((1 << RemoteTrimPacket::valueLengthBits) - 1);
		};

		ac.settings.motors.aileronsTrim = read();
		ac.settings.motors.elevatorTrim = read();
		ac.settings.motors.rudderTrim = read();
		ac.settings.motors.scheduleWrite();
		
		ac.fbw.applyData();
		
		return true;
	}
	
	bool AircraftPacketHandler::receiveRemoteLightsPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			4,
			payloadLength
		))
			return false;
		
		ac.lights.setNavigationEnabled(stream.readBool());
		ac.lights.setStrobeEnabled(stream.readBool());
		ac.lights.setLandingEnabled(stream.readBool());
		ac.lights.setCabinEnabled(stream.readBool());
		
		return true;
	}
	
	bool AircraftPacketHandler::receiveRemoteBaroPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteBaroPacket::referencePressureLengthBits,
			payloadLength
		))
			return false;
		
		// Reference pressure
		const auto referencePressureDaPa = stream.readUint16(RemoteBaroPacket::referencePressureLengthBits);
		ac.adirs.setReferencePressurePa(sanitizeValue<uint32_t>(static_cast<uint32_t>(referencePressureDaPa) * 10, 900'00, 1100'00));
		
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
	
	bool AircraftPacketHandler::receiveRemoteAutopilotPacket(BitStream& stream, uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
				+ RemoteAutopilotPacket::speedLengthBits
				+ RemoteAutopilotPacket::headingLengthBits
				+ RemoteAutopilotPacket::altitudeLengthBits
				+ RemoteAutopilotPacket::lateralModeLengthBits
				+ RemoteAutopilotPacket::verticalModeLengthBits
				// A/T
				+ 1
				// A/P
				+ 1,
			payloadLength
		))
			return false;
		
		// Speed
		const auto speedFactor =
			static_cast<float>(stream.readUint8(RemoteAutopilotPacket::speedLengthBits))
			/ static_cast<float>((1 << RemoteAutopilotPacket::speedLengthBits) - 1);
		
		ac.fbw.setSelectedSpeedMps(static_cast<float>(RemoteAutopilotPacket::speedMaxMPS) * speedFactor);
		
		// Heading
		ac.fbw.setSelectedHeadingDeg(stream.readUint16(RemoteAutopilotPacket::headingLengthBits));
		
		// Altitude
		ac.fbw.setSelectedAltitudeM(readAltitude(
			stream,
			RemoteAutopilotPacket::altitudeLengthBits,
			RemoteAutopilotPacket::altitudeMinM,
			RemoteAutopilotPacket::altitudeMaxM
		));
		
		// Modes
		ac.fbw.setLateralMode(static_cast<AutopilotLateralMode>(stream.readUint8(RemoteAutopilotPacket::lateralModeLengthBits)));
		ac.fbw.setVerticalMode(static_cast<AutopilotVerticalMode>(stream.readUint8(RemoteAutopilotPacket::verticalModeLengthBits)));
		
		// Autothrottle
		ac.fbw.setAutothrottle(stream.readBool());
		
		// Autopilot
		ac.fbw.setAutopilot(stream.readBool());
		
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
			std::min<float>(ac.adirs.getAccelSpeedMPS(), AircraftADIRSPacket::speedMaxMPS)
		    / static_cast<float>(AircraftADIRSPacket::speedMaxMPS);
		
		const auto speedMapped = static_cast<float>((1 << AircraftADIRSPacket::speedLengthBits) - 1) * speedFactor;
		
		stream.writeUint8(static_cast<uint8_t>(speedMapped), AircraftADIRSPacket::speedLengthBits);
		
		// Altitude
		writeAltitude(
			stream,
			ac.adirs.getCoordinates().getAltitude(),
			AircraftADIRSPacket::altitudeLengthBits,
			AircraftADIRSPacket::altitudeMinM,
			AircraftADIRSPacket::altitudeMaxM
		);
		
		// -------------------------------- Autopilot --------------------------------
		
		// Roll
		writeRadians(stream, ac.fbw.getTargetRollRad(), AircraftADIRSPacket::autopilotRollRangeRad, AircraftADIRSPacket::autopilotRollLengthBits);
		
		// Pitch
		writeRadians(stream, ac.fbw.getTargetPitchRad(), AircraftADIRSPacket::autopilotPitchRangeRad, AircraftADIRSPacket::autopilotPitchLengthBits);
		
		return true;
	}
	
	bool AircraftPacketHandler::transmitAircraftAuxiliaryPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		const auto& coordinates = ac.adirs.getCoordinates();
		
		// -------------------------------- Throttle --------------------------------
		
		const auto motor = ac.motors.getMotor(MotorType::throttle);
		
		stream.writeUint8(
			(motor ? motor->getPower() : 0) * ((1 << AircraftAuxiliaryPacket::throttleLengthBits) - 1) / Motor::powerMaxValue,
			AircraftAuxiliaryPacket::throttleLengthBits
		);
		
		// -------------------------------- Lat / lon --------------------------------
		
		// Lat
		const auto latRad = coordinates.getLatitude();
		// Mapping from [-90; 90] to [0; 180] and then to [0; 1]
		const auto latFactor = (latRad + std::numbers::pi_v<float> / 2.f) / std::numbers::pi_v<float>;
		const auto latValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftAuxiliaryPacket::latLengthBits) - 1) * latFactor);
		
		stream.writeUint32(latValue, AircraftAuxiliaryPacket::latLengthBits);
		
		// Lon
		const auto lonRad = coordinates.getLongitude();
		// Mapping from [0; 360] to [0; 1]
		const auto lonFactor = lonRad / (2 * std::numbers::pi_v<float>);
		const auto lonValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftAuxiliaryPacket::lonLengthBits) - 1) * lonFactor);
		
		stream.writeUint32(lonValue, AircraftAuxiliaryPacket::lonLengthBits);
		
		// -------------------------------- Battery --------------------------------
		
		stream.writeUint16(125, AircraftAuxiliaryPacket::batteryLengthBits);
		
		// -------------------------------- Lights --------------------------------
		
		stream.writeBool(ac.lights.isNavigationEnabled());
		stream.writeBool(ac.lights.isStrobeEnabled());
		stream.writeBool(ac.lights.isLandingEnabled());
		stream.writeBool(ac.lights.isCabinEnabled());
		
		// -------------------------------- Autopilot --------------------------------
		
		// Modes
		stream.writeUint8(std::to_underlying(ac.fbw.getLateralMode()), AircraftAuxiliaryPacket::autopilotLateralModeLengthBits);
		stream.writeUint8(std::to_underlying(ac.fbw.getVerticalMode()), AircraftAuxiliaryPacket::autopilotVerticalModeLengthBits);
		
		// Altitude for ALT/VNAV modes
		writeAltitude(
			stream,
			ac.fbw.getSelectedAltitudeM(),
			AircraftAuxiliaryPacket::autopilotAltitudeLengthBits,
			AircraftAuxiliaryPacket::autopilotAltitudeMinM,
			AircraftAuxiliaryPacket::autopilotAltitudeMaxM
		);
		
		// Autothrottle
		stream.writeBool(ac.fbw.getAutothrottle());
		
		// Autopilot
		stream.writeBool(ac.fbw.getAutopilot());
		
		return true;
	}
}