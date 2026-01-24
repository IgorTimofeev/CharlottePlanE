#include "../types/aircraftCommunicationManager.h"

#include <utility>

#include "aircraft.h"
#include "systems/motors.h"

namespace YOBA {
	// -------------------------------- PacketSequenceItem --------------------------------
	
	PacketSequenceItem::PacketSequenceItem(const AircraftPacketType type, const uint8_t count, const bool useEnqueued) : _type(type), _count(count), _useEnqueued(useEnqueued) {
	
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
	
	void AircraftCommunicationManager::onStart() {
		ESP_LOGI(_logTag, "started");
		
		while (true) {
			if (receive(1'000'000)) {
				vTaskDelay(pdMS_TO_TICKS(20));
				transmit(1'000'000);
			}
		}
	}
	
	void AircraftCommunicationManager::enqueue(const AircraftPacketType type) {
		_enqueuedPackets.insert(type);
	}
	
	void AircraftCommunicationManager::onConnectionStateChanged() {
		auto& ac = Aircraft::getInstance();
		
		if (isConnected()) {
			ac.lights.setEmergencyEnabled(false);
		}
		else {
			ac.lights.setEmergencyEnabled(true);
		}
	}
	
	// -------------------------------- Receiving --------------------------------
	
	bool AircraftCommunicationManager::onReceive(BitStream& stream, const RemotePacketType packetType, const uint8_t payloadLength) {
		switch (packetType) {
			case RemotePacketType::controls:
				return receiveRemoteControlsPacket(stream, payloadLength);
			
			case RemotePacketType::trim:
				return receiveRemoteTrimPacket(stream, payloadLength);
			
			case RemotePacketType::lights:
				return receiveRemoteLightsPacket(stream, payloadLength);
			
			case RemotePacketType::baro:
				return receiveRemoteBaroPacket(stream, payloadLength);
			
			case RemotePacketType::calibrate:
				return receiveRemoteCalibratePacket(stream, payloadLength);
			
			case RemotePacketType::autopilot:
				return receiveRemoteAutopilotPacket(stream, payloadLength);
			
			case RemotePacketType::motorConfiguration:
				return receiveMotorConfigurationPacket(stream, payloadLength);
			
			default:
				ESP_LOGE(_logTag, "failed to receive packet: unsupported type %d", std::to_underlying(packetType));
				return false;
		}
		
		return true;
	}
	
	bool AircraftCommunicationManager::receiveRemoteControlsPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteControlsPacket::motorLengthBits * 6,
			payloadLength
		))
			return false;
		
		const auto readMotor = [&stream] {
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
	
	bool AircraftCommunicationManager::receiveRemoteTrimPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteTrimPacket::valueLengthBits * 3,
			payloadLength
		))
			return false;
		
		const auto read = [&stream] {
			return
				// Mapping [0; bits] to [-0.5; 0.5]
				static_cast<float>(stream.readUint16(RemoteTrimPacket::valueLengthBits))
				/ static_cast<float>((1 << RemoteTrimPacket::valueLengthBits) - 1)
				- 0.5f;
		};

		ac.settings.trim.aileronsTrim = read();
		ac.settings.trim.elevatorTrim = read();
		ac.settings.trim.rudderTrim = read();
		ac.settings.trim.scheduleWrite();
		
		ac.fbw.applyData();
		
		return true;
	}
	
	bool AircraftCommunicationManager::receiveRemoteLightsPacket(BitStream& stream, const uint8_t payloadLength) {
		const auto& ac = Aircraft::getInstance();
		
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
	
	bool AircraftCommunicationManager::receiveRemoteBaroPacket(BitStream& stream, const uint8_t payloadLength) {
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
	
	bool AircraftCommunicationManager::receiveRemoteAutopilotPacket(BitStream& stream, const uint8_t payloadLength) {
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
	
	bool AircraftCommunicationManager::receiveRemoteCalibratePacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		ESP_LOGI(_logTag, "R!!!!ket");
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteCalibratePacket::systemLengthBits,
			payloadLength
		))
			return false;
		
		ac.aircraftData.calibration.system = static_cast<AircraftCalibrationSystem>(stream.readUint8(RemoteCalibratePacket::systemLengthBits));
		ac.aircraftData.calibration.progress = 0;
		ac.aircraftData.calibration.calibrating = true;

//		ESP_LOGI(_logTag, "Received calibrate packet");
		
		return true;
	}
	
	bool AircraftCommunicationManager::receiveMotorConfigurationPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		ESP_LOGI(_logTag, "Received motor config packet");
		
		if (!validatePayloadChecksumAndLength(
			stream,
			(
				RemoteMotorConfigurationPacket::minLengthBits
				+ RemoteMotorConfigurationPacket::maxLengthBits
				+ 1
			)
			* 8,
			payloadLength
		))
			return false;
		
		const auto read = [&stream](MotorConfiguration& configuration) {
			configuration.min = stream.readUint16(RemoteMotorConfigurationPacket::minLengthBits);
			configuration.max = stream.readUint16(RemoteMotorConfigurationPacket::maxLengthBits);
			configuration.reverse = stream.readBool();
			configuration.sanitize();
			
			ESP_LOGI(_logTag, "min: %d, max: %d, reverse: %d", configuration.min, configuration.max, configuration.reverse);
		};
		
		read(ac.settings.motors.throttle);
		read(ac.settings.motors.noseWheel);
		
		read(ac.settings.motors.flapLeft);
		read(ac.settings.motors.aileronLeft);

		read(ac.settings.motors.flapRight);
		read(ac.settings.motors.aileronRight);

		read(ac.settings.motors.tailLeft);
		read(ac.settings.motors.tailRight);
		
		ac.motors.updateConfigurationsFromSettings();
		ac.settings.motors.scheduleWrite();
		
		return true;
	}
	
	// -------------------------------- Transmitting --------------------------------
	
	AircraftPacketType AircraftCommunicationManager::getTransmitPacketType() {
		const auto& item = _packetSequence[_packetSequenceIndex];
		
		const auto next = [this, &item] {
			_packetSequenceItemCounter++;
			
			if (_packetSequenceItemCounter < item.getCount())
				return;
			
			_packetSequenceItemCounter = 0;
			
			_packetSequenceIndex++;
			
			if (_packetSequenceIndex >= _packetSequence.size())
				_packetSequenceIndex = 0;
		};
		
		// Enqueued
		if (item.useEnqueued() && !_enqueuedPackets.empty()) {
			const auto packetType = *_enqueuedPackets.begin();
			_enqueuedPackets.erase(packetType);
			
			next();
			
			return packetType;
		}

		// Normal
		const auto packetType = item.getType();
			
		next();
			
		return packetType;
	}
	
	bool AircraftCommunicationManager::onTransmit(BitStream& stream, const AircraftPacketType packetType) {
		switch (packetType) {
			case AircraftPacketType::ADIRS:
				transmitAircraftADIRSPacket(stream);
				break;
			
			case AircraftPacketType::auxiliary:
				transmitAircraftAuxiliaryPacket(stream);
				break;
			
			case AircraftPacketType::calibration:
				transmitAircraftCalibrationPacket(stream);
				break;
			
			default:
				ESP_LOGE(_logTag, "failed to write packet: unsupported type %d", packetType);
				
				return false;
		}
		
		return true;
	}
	
	void AircraftCommunicationManager::transmitAircraftADIRSPacket(BitStream& stream) {
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
	}
	
	void AircraftCommunicationManager::transmitAircraftAuxiliaryPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		const auto& coordinates = ac.adirs.getCoordinates();
		
		// -------------------------------- Throttle --------------------------------
		
		const auto motor = ac.motors.getMotor(MotorType::throttle);
		
		stream.writeUint8(
			(motor ? motor->getPower() : 0) * ((1 << AircraftAuxiliaryPacket::throttleLengthBits) - 1) / Motor::powerMax,
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

		// Decavolts
		stream.writeUint16(ac.battery.getVoltage() / 100, AircraftAuxiliaryPacket::batteryLengthBits);
		
		// -------------------------------- Lights --------------------------------
		
		stream.writeBool(ac.settings.lights.nav);
		stream.writeBool(ac.settings.lights.strobe);
		stream.writeBool(ac.settings.lights.landing);
		stream.writeBool(ac.settings.lights.cabin);
		
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
	}
	
	void AircraftCommunicationManager::transmitAircraftCalibrationPacket(BitStream& stream) {
		const auto& ac = Aircraft::getInstance();
	
		stream.writeUint8(std::to_underlying(ac.aircraftData.calibration.system), AircraftCalibrationPacket::systemLengthBits);
		stream.writeUint8(ac.aircraftData.calibration.progress * ((1 << AircraftCalibrationPacket::progressLengthBits) - 1) / 0xFF, AircraftCalibrationPacket::progressLengthBits);
	}
}