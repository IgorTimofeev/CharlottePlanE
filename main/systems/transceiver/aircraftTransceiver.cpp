#include "systems/transceiver/aircraftTransceiver.h"

#include <utility>

#include "aircraft.h"
#include "systems/motors/motors.h"

namespace pizda {
	// -------------------------------- Generic --------------------------------

	AircraftTransceiver::AircraftTransceiver() : Transceiver({
		PacketSequenceItem(AircraftPacketType::telemetryPrimary, 3),
		PacketSequenceItem(AircraftPacketType::telemetryPrimary, 1, true),
		PacketSequenceItem(AircraftPacketType::telemetrySecondary, 1)
	}) {

	}

	void AircraftTransceiver::onStart() {
		// Receive -> wait -> transmit

		bool receiveMode = true;
		int64_t transmitTime = 0;

		while (true) {
			if (receiveMode) {
				if (receive(1'000'000)) {
					receiveMode = false;

					transmitTime = esp_timer_get_time() + 8'000;
				}
			}
			else {
				if (esp_timer_get_time() >= transmitTime) {
					transmit(1'000'000);
					receiveMode = true;
				}
				else {
					taskYIELD();
				}
			}
		}
	}

	void AircraftTransceiver::onConnectionStateChanged() {
		auto& ac = Aircraft::getInstance();
		
		if (isConnected()) {
			ac.lights.setEmergencyEnabled(false);
		}
		else {
			ac.lights.setEmergencyEnabled(true);
		}
	}
	
	// -------------------------------- Receiving --------------------------------
	
	bool AircraftTransceiver::onReceive(BitStream& stream, const RemotePacketType packetType, const uint8_t payloadLength) {
		switch (packetType) {
			case RemotePacketType::controls:
				return receiveRemoteControlsPacket(stream, payloadLength);
			
			case RemotePacketType::auxiliary:
				return receiveRemoteAuxiliaryPacket(stream, payloadLength);

			default:
				ESP_LOGE(_logTag, "failed to receive packet: unsupported type %d", std::to_underlying(packetType));
				return false;
		}
		
		return true;
	}


	bool AircraftTransceiver::receiveRemoteControlsPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();

		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteControlsPacket::motorLengthBits * 5,
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

		return true;
	}

	bool AircraftTransceiver::receiveRemoteAuxiliaryPacket(BitStream& stream, const uint8_t payloadLength) {
		const auto type = static_cast<RemoteAuxiliaryPacketType>(stream.readUint8(RemoteAuxiliaryPacket::typeLengthBits));

		switch (type) {
			case RemoteAuxiliaryPacketType::trim:
				return receiveRemoteAuxiliaryTrimPacket(stream, payloadLength);

			case RemoteAuxiliaryPacketType::lights:
				return receiveRemoteAuxiliaryLightsPacket(stream, payloadLength);

			case RemoteAuxiliaryPacketType::baro:
				return receiveRemoteAuxiliaryBaroPacket(stream, payloadLength);

			case RemoteAuxiliaryPacketType::calibrate:
				return receiveRemoteAuxiliaryCalibratePacket(stream, payloadLength);

			case RemoteAuxiliaryPacketType::autopilot:
				return receiveRemoteAuxiliaryAutopilotPacket(stream, payloadLength);

			case RemoteAuxiliaryPacketType::motorConfiguration:
				return receiveRemoteAuxiliaryMotorConfigurationPacket(stream, payloadLength);

			default:
				ESP_LOGE(_logTag, "failed to receive packet: unsupported type %d", std::to_underlying(type));
				return false;
		}
	}
	
	bool AircraftTransceiver::receiveRemoteAuxiliaryTrimPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteAuxiliaryPacket::typeLengthBits
				+ RemoteAuxiliaryTrimPacket::valueLengthBits * 3,
			payloadLength
		))
			return false;
		
		const auto read = [&stream] {
			return
				// Mapping [0; bits] to [-0.5; 0.5]
				static_cast<float>(stream.readUint16(RemoteAuxiliaryTrimPacket::valueLengthBits))
				/ static_cast<float>((1 << RemoteAuxiliaryTrimPacket::valueLengthBits) - 1)
				- 0.5f;
		};

		ac.settings.trim.aileronsTrim = read();
		ac.settings.trim.elevatorTrim = read();
		ac.settings.trim.rudderTrim = read();
		ac.settings.trim.scheduleWrite();

		return true;
	}
	
	bool AircraftTransceiver::receiveRemoteAuxiliaryLightsPacket(BitStream& stream, const uint8_t payloadLength) {
		const auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteAuxiliaryPacket::typeLengthBits
				+ 4,
			payloadLength
		))
			return false;
		
		ac.lights.setNavigationEnabled(stream.readBool());
		ac.lights.setStrobeEnabled(stream.readBool());
		ac.lights.setLandingEnabled(stream.readBool());
		ac.lights.setCabinEnabled(stream.readBool());
		
		return true;
	}
	
	bool AircraftTransceiver::receiveRemoteAuxiliaryBaroPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteAuxiliaryPacket::typeLengthBits
				+ RemoteAuxiliaryBaroPacket::referencePressureLengthBits,
			payloadLength
		))
			return false;
		
		// Reference pressure
		const auto referencePressureDaPa = stream.readUint16(RemoteAuxiliaryBaroPacket::referencePressureLengthBits);

		ac.settings.adirs.referencePressurePa = sanitizeValue<uint32_t>(static_cast<uint32_t>(referencePressureDaPa) * 10, 900'00, 1100'00);
		ac.settings.adirs.scheduleWrite();

		ac.adirs.setReferencePressurePa(ac.settings.adirs.referencePressurePa);
		
		return true;
	}
	
	bool AircraftTransceiver::receiveRemoteAuxiliaryAutopilotPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteAuxiliaryPacket::typeLengthBits
				+ RemoteAuxiliaryAutopilotPacket::speedLengthBits
				+ RemoteAuxiliaryAutopilotPacket::headingLengthBits
				+ RemoteAuxiliaryAutopilotPacket::altitudeLengthBits
				+ RemoteAuxiliaryAutopilotPacket::lateralModeLengthBits
				+ RemoteAuxiliaryAutopilotPacket::verticalModeLengthBits
				// A/T
				+ 1
				// A/P
				+ 1,
			payloadLength
		))
			return false;
		
		// Speed
		const auto speedFactor =
			static_cast<float>(stream.readUint8(RemoteAuxiliaryAutopilotPacket::speedLengthBits))
			/ static_cast<float>((1 << RemoteAuxiliaryAutopilotPacket::speedLengthBits) - 1);
		
		ac.fbw.setSelectedSpeedMps(static_cast<float>(RemoteAuxiliaryAutopilotPacket::speedMaxMPS) * speedFactor);
		
		// Heading
		ac.fbw.setSelectedHeadingDeg(stream.readUint16(RemoteAuxiliaryAutopilotPacket::headingLengthBits));
		
		// Altitude
		ac.fbw.setSelectedAltitudeM(readAltitude(
			stream,
			RemoteAuxiliaryAutopilotPacket::altitudeLengthBits,
			RemoteAuxiliaryAutopilotPacket::altitudeMinM,
			RemoteAuxiliaryAutopilotPacket::altitudeMaxM
		));
		
		// Modes
		ac.fbw.setLateralMode(static_cast<AutopilotLateralMode>(stream.readUint8(RemoteAuxiliaryAutopilotPacket::lateralModeLengthBits)));
		ac.fbw.setVerticalMode(static_cast<AutopilotVerticalMode>(stream.readUint8(RemoteAuxiliaryAutopilotPacket::verticalModeLengthBits)));
		
		// Autothrottle
		ac.fbw.setAutothrottle(stream.readBool());
		
		// Autopilot
		ac.fbw.setAutopilot(stream.readBool());
		
		return true;
	}
	
	bool AircraftTransceiver::receiveRemoteAuxiliaryCalibratePacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteAuxiliaryPacket::typeLengthBits
				+ RemoteAuxiliaryCalibratePacket::systemLengthBits,
			payloadLength
		))
			return false;
		
		ac.aircraftData.calibration.system = static_cast<AircraftCalibrationSystem>(stream.readUint8(RemoteAuxiliaryCalibratePacket::systemLengthBits));
		ac.aircraftData.calibration.progress = 0;
		ac.aircraftData.calibration.calibrating = true;

//		ESP_LOGI(_logTag, "Received calibrate packet");
		
		return true;
	}
	
	bool AircraftTransceiver::receiveRemoteAuxiliaryMotorConfigurationPacket(BitStream& stream, const uint8_t payloadLength) {
		auto& ac = Aircraft::getInstance();
		
		ESP_LOGI(_logTag, "Received motor config packet");
		
		if (!validatePayloadChecksumAndLength(
			stream,
			RemoteAuxiliaryPacket::typeLengthBits
				+ (
					RemoteAuxiliaryMotorConfigurationPacket::minLengthBits
					+ RemoteAuxiliaryMotorConfigurationPacket::maxLengthBits
					+ 1
				)
				* 8,
			payloadLength
		))
			return false;
		
		const auto read = [&stream](MotorConfiguration& configuration) {
			configuration.min = stream.readUint16(RemoteAuxiliaryMotorConfigurationPacket::minLengthBits);
			configuration.max = stream.readUint16(RemoteAuxiliaryMotorConfigurationPacket::maxLengthBits);
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
	
	void AircraftTransceiver::onTransmit(BitStream& stream, const AircraftPacketType packetType) {
		switch (packetType) {
			case AircraftPacketType::telemetryPrimary:
				transmitAircraftTelemetryPrimaryPacket(stream);
				break;
			
			case AircraftPacketType::telemetrySecondary:
				transmitAircraftTelemetrySecondaryPacket(stream);
				break;
			
			case AircraftPacketType::auxiliary:
				transmitAircraftAuxiliaryPacket(stream);
				break;
			
			default:
				ESP_LOGE(_logTag, "failed to write packet: unsupported type %d", packetType);
				break;
		}
	}
	
	void AircraftTransceiver::transmitAircraftTelemetryPrimaryPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		// Roll / pitch / yaw
		writeRadians(stream, ac.adirs.getRollRad(), 2.f * std::numbers::pi_v<float>, AircraftTelemetryPrimaryPacket::rollLengthBits);
		writeRadians(stream, ac.adirs.getPitchRad(), std::numbers::pi_v<float>, AircraftTelemetryPrimaryPacket::pitchLengthBits);
		writeRadians(stream, ac.adirs.getYawRad(), 2.f * std::numbers::pi_v<float>, AircraftTelemetryPrimaryPacket::yawLengthBits);

		// Slip & skid
		const auto slipAndSkidValue = static_cast<uint8_t>(
			static_cast<float>((1 << AircraftTelemetryPrimaryPacket::slipAndSkidLengthBits) - 1)
			// Mapping from [-1.0; 1.0] to [0.0; 1.0]
			* (ac.adirs.getSlipAndSkidFactor() + 1.f) / 2.f
		);
		
		stream.writeUint8(slipAndSkidValue, AircraftTelemetryPrimaryPacket::slipAndSkidLengthBits);
		
		// Speed
		const auto speedFactor =
			std::min<float>(ac.adirs.getAirspeedMPS(), AircraftTelemetryPrimaryPacket::speedMaxMPS)
		    / static_cast<float>(AircraftTelemetryPrimaryPacket::speedMaxMPS);
		
		const auto speedMapped = static_cast<float>((1 << AircraftTelemetryPrimaryPacket::speedLengthBits) - 1) * speedFactor;
		
		stream.writeUint8(static_cast<uint8_t>(speedMapped), AircraftTelemetryPrimaryPacket::speedLengthBits);
		
		// Altitude
		writeAltitude(
			stream,
			ac.adirs.getCoordinates().getAltitude(),
			AircraftTelemetryPrimaryPacket::altitudeLengthBits,
			AircraftTelemetryPrimaryPacket::altitudeMinM,
			AircraftTelemetryPrimaryPacket::altitudeMaxM
		);
		
		// -------------------------------- Autopilot --------------------------------
		
		// Roll
		writeRadians(stream, ac.fbw.getTargetRollRad(), AircraftTelemetryPrimaryPacket::autopilotRollRangeRad, AircraftTelemetryPrimaryPacket::autopilotRollLengthBits);
		
		// Pitch
		writeRadians(stream, ac.fbw.getTargetPitchRad(), AircraftTelemetryPrimaryPacket::autopilotPitchRangeRad, AircraftTelemetryPrimaryPacket::autopilotPitchLengthBits);
	}
	
	void AircraftTransceiver::transmitAircraftTelemetrySecondaryPacket(BitStream& stream) {
		auto& ac = Aircraft::getInstance();
		
		const auto& coordinates = ac.adirs.getCoordinates();
		
		// -------------------------------- Throttle --------------------------------
		
		const auto motor = ac.motors.getMotor(MotorType::throttle);
		
		stream.writeUint8(
			(motor ? motor->getPower() : 0) * ((1 << AircraftTelemetrySecondaryPacket::throttleLengthBits) - 1) / Motor::powerMax,
			AircraftTelemetrySecondaryPacket::throttleLengthBits
		);
		
		// -------------------------------- Lat / lon --------------------------------
		
		// Lat
		const auto latRad = coordinates.getLatitude();
		// Mapping from [-90; 90] to [0; 180] and then to [0; 1]
		const auto latFactor = (latRad + std::numbers::pi_v<float> / 2.f) / std::numbers::pi_v<float>;
		const auto latValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftTelemetrySecondaryPacket::latLengthBits) - 1) * latFactor);
		
		stream.writeUint32(latValue, AircraftTelemetrySecondaryPacket::latLengthBits);
		
		// Lon
		const auto lonRad = coordinates.getLongitude();
		// Mapping from [0; 360] to [0; 1]
		const auto lonFactor = lonRad / (2 * std::numbers::pi_v<float>);
		const auto lonValue = static_cast<uint32_t>(static_cast<float>((1 << AircraftTelemetrySecondaryPacket::lonLengthBits) - 1) * lonFactor);
		
		stream.writeUint32(lonValue, AircraftTelemetrySecondaryPacket::lonLengthBits);
		
		// -------------------------------- Battery --------------------------------

		// Decavolts
		stream.writeUint16(ac.battery.getVoltage() / 100, AircraftTelemetrySecondaryPacket::batteryLengthBits);
		
		// -------------------------------- Lights --------------------------------
		
		stream.writeBool(ac.settings.lights.nav);
		stream.writeBool(ac.settings.lights.strobe);
		stream.writeBool(ac.settings.lights.landing);
		stream.writeBool(ac.settings.lights.cabin);
		
		// -------------------------------- Autopilot --------------------------------
		
		// Modes
		stream.writeUint8(std::to_underlying(ac.fbw.getLateralMode()), AircraftTelemetrySecondaryPacket::autopilotLateralModeLengthBits);
		stream.writeUint8(std::to_underlying(ac.fbw.getVerticalMode()), AircraftTelemetrySecondaryPacket::autopilotVerticalModeLengthBits);
		
		// Altitude for ALT/VNAV modes
		writeAltitude(
			stream,
			ac.fbw.getSelectedAltitudeM(),
			AircraftTelemetrySecondaryPacket::autopilotAltitudeLengthBits,
			AircraftTelemetrySecondaryPacket::autopilotAltitudeMinM,
			AircraftTelemetrySecondaryPacket::autopilotAltitudeMaxM
		);
		
		// Autothrottle
		stream.writeBool(ac.fbw.getAutothrottle());
		
		// Autopilot
		stream.writeBool(ac.fbw.getAutopilot());
	}

	void AircraftTransceiver::transmitAircraftAuxiliaryPacket(BitStream& stream) {
		stream.writeUint8(std::to_underlying(getEnqueuedAuxiliaryPacketType()), AircraftAuxiliaryPacket::typeLengthBits);

		switch (getEnqueuedAuxiliaryPacketType()) {
			case AircraftAuxiliaryPacketType::calibration:
				transmitAircraftAuxiliaryCalibrationPacket(stream);
				break;
		}
	}

	void AircraftTransceiver::transmitAircraftAuxiliaryCalibrationPacket(BitStream& stream) {
		const auto& ac = Aircraft::getInstance();
	
		stream.writeUint8(std::to_underlying(ac.aircraftData.calibration.system), AircraftAuxiliaryCalibrationPacket::systemLengthBits);
		stream.writeUint8(static_cast<uint16_t>(ac.aircraftData.calibration.progress) * ((1 << AircraftAuxiliaryCalibrationPacket::progressLengthBits) - 1) / 0xFF, AircraftAuxiliaryCalibrationPacket::progressLengthBits);
	}
}