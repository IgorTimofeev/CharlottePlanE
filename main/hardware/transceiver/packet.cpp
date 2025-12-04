#include "packet.h"

#include <cmath>
#include "settings.h"
#include "rc.h"

namespace pizda {
	uint8_t Packet::getCRC8(const uint8_t* data, size_t length) {
		uint8_t crc = 0xff;
		size_t i, j;

		for (i = 0; i < length; i++) {
			crc ^= data[i];

			for (j = 0; j < 8; j++) {
				if ((crc & 0x80) != 0)
					crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
				else
					crc <<= 1;
			}
		}

		return crc;
	}

	 bool Packet::checkCRC(const uint8_t* data, size_t length) {
		auto checksum = getCRC8(data, length);
		auto expectedChecksum = *(data + length);

//		ESP_LOGI("Packet", "Checksum: %d, expected: %d", checksum, expectedChecksum);

		if (checksum != expectedChecksum) {
			ESP_LOGE("Packet", "Checksum mismatch: got %d, expected %d", checksum, expectedChecksum);

			return false;
		}

		return true;
	}

	void Packet::parse(uint8_t* buffer) {
		auto& rc = RC::getInstance();

		// Header
		if (memcmp(buffer, Packet::header, Packet::headerLength) != 0) {
			ESP_LOGE("Packet", "Mismatched header: %s", buffer);

			return;
		}

		buffer += Packet::headerLength;

		// Type
		const auto dataType = *reinterpret_cast<PacketType*>(buffer);
		buffer += 1;

		ESP_LOGI("Packet", "Data type: %d", static_cast<uint8_t>(dataType));

		switch (dataType) {
			case PacketType::RemoteSetControlsValues: {
				ESP_LOGI("Packet", "-------- RemoteSetControlsValues --------");

				const auto controlsCount = *buffer;

				ESP_LOGI("Packet", "Controls count: %d", controlsCount);

				if (!checkCRC(buffer, 1 + sizeof(uint16_t) * controlsCount))
					return;

				buffer++;

				// Updating motors position
				rc.motors.setLeftThrottle(*reinterpret_cast<uint16_t*>(buffer));
				buffer += sizeof(uint16_t);

				rc.motors.setRightThrottle(*reinterpret_cast<uint16_t*>(buffer));
				buffer += sizeof(uint16_t);

				rc.motors.setAilerons(*reinterpret_cast<uint16_t*>(buffer));
				buffer += sizeof(uint16_t);

				rc.motors.setElevator(*reinterpret_cast<uint16_t*>(buffer));
				buffer += sizeof(uint16_t);

				rc.motors.setRudder(*reinterpret_cast<uint16_t*>(buffer));
				buffer += sizeof(uint16_t);

				rc.motors.setFlaps(*reinterpret_cast<uint16_t*>(buffer));
				buffer += sizeof(uint16_t);

				ESP_LOGI("Packet", "Left throttle: %d", rc.motors.getLeftThrottle());
				ESP_LOGI("Packet", "Right throttle: %d", rc.motors.getRightThrottle());
				ESP_LOGI("Packet", "Ailerons: %d", rc.motors.getAilerons());
				ESP_LOGI("Packet", "Elevator: %d", rc.motors.getElevator());
				ESP_LOGI("Packet", "Rudder: %d", rc.motors.getRudder());
				ESP_LOGI("Packet", "Flaps: %d", rc.motors.getFlaps());
				ESP_LOGI("Packet", "----------------");

				break;
			}
			case PacketType::RemoteSetMotorConfigurations: {
				ESP_LOGI("Packet", "-------- RemoteSetMotorConfigurations --------");

				const auto motorCount = *buffer;

				ESP_LOGI("Packet", "Motor count: %d", motorCount);

				if (!checkCRC(buffer, 1 + sizeof(MotorSettings) * motorCount))
					return;

				buffer++;

				// Saving new calibration data
				rc.settings.motors.leftThrottle = *reinterpret_cast<MotorSettings*>(buffer);
				rc.settings.motors.leftThrottle.sanitize();
				buffer += sizeof(MotorSettings);

				rc.settings.motors.rightThrottle = *reinterpret_cast<MotorSettings*>(buffer);
				rc.settings.motors.rightThrottle.sanitize();
				buffer += sizeof(MotorSettings);

				rc.settings.motors.leftAileron = *reinterpret_cast<MotorSettings*>(buffer);
				rc.settings.motors.leftAileron.sanitize();
				buffer += sizeof(MotorSettings);

				rc.settings.motors.rightAileron = *reinterpret_cast<MotorSettings*>(buffer);
				rc.settings.motors.rightAileron.sanitize();
				buffer += sizeof(MotorSettings);

				rc.settings.motors.leftTail = *reinterpret_cast<MotorSettings*>(buffer);
				rc.settings.motors.leftTail.sanitize();
				buffer += sizeof(MotorSettings);

				rc.settings.motors.rightTail = *reinterpret_cast<MotorSettings*>(buffer);
				rc.settings.motors.rightTail.sanitize();
				buffer += sizeof(MotorSettings);

				rc.settings.motors.leftFlap = *reinterpret_cast<MotorSettings*>(buffer);
				rc.settings.motors.leftFlap.sanitize();
				buffer += sizeof(MotorSettings);

				rc.settings.motors.rightFlap = *reinterpret_cast<MotorSettings*>(buffer);
				rc.settings.motors.rightFlap.sanitize();
				buffer += sizeof(MotorSettings);

//				rc.settings.motors.scheduleWrite();

				// Updating motors position
				rc.motors.setAilerons(rc.motors.getAilerons());
				rc.motors.setFlaps(rc.motors.getFlaps());

				ESP_LOGI("Packet", "Left throttle min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftThrottle.min, rc.settings.motors.leftThrottle.max, rc.settings.motors.leftThrottle.offset, rc.settings.motors.leftThrottle.reverse);
				ESP_LOGI("Packet", "Right throttle min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightThrottle.min, rc.settings.motors.rightThrottle.max, rc.settings.motors.rightThrottle.offset, rc.settings.motors.rightThrottle.reverse);
				ESP_LOGI("Packet", "Left aileron min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftAileron.min, rc.settings.motors.leftAileron.max, rc.settings.motors.leftAileron.offset, rc.settings.motors.leftAileron.reverse);
				ESP_LOGI("Packet", "Right aileron min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightAileron.min, rc.settings.motors.rightAileron.max, rc.settings.motors.rightAileron.offset, rc.settings.motors.rightAileron.reverse);
				ESP_LOGI("Packet", "Left tail min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftTail.min, rc.settings.motors.leftTail.max, rc.settings.motors.leftTail.offset, rc.settings.motors.leftTail.reverse);
				ESP_LOGI("Packet", "Right tail min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightTail.min, rc.settings.motors.rightTail.max, rc.settings.motors.rightTail.offset, rc.settings.motors.rightTail.reverse);
				ESP_LOGI("Packet", "Left flap min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftFlap.min, rc.settings.motors.leftFlap.max, rc.settings.motors.leftFlap.offset, rc.settings.motors.leftFlap.reverse);
				ESP_LOGI("Packet", "Right flap min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightFlap.min, rc.settings.motors.rightFlap.max, rc.settings.motors.rightFlap.offset, rc.settings.motors.rightFlap.reverse);
				ESP_LOGI("Packet", "----------------");

				break;
			}
			case PacketType::RemoteSetBooleanValues: {
				ESP_LOGI("Packet", "-------- RemoteSetBooleanValues --------");

				const auto booleanCount = *buffer;

				ESP_LOGI("Packet", "Boolean count: %d", booleanCount);

				if (!checkCRC(buffer, 1 + booleanCount))
					return;

				buffer++;

				rc.lights.setNavigationEnabled((*buffer >> 0) & 0b1);
				rc.lights.setStrobeEnabled((*buffer >> 1) & 0b1);
				rc.lights.setLandingEnabled((*buffer >> 2) & 0b1);
				rc.lights.setCabinEnabled((*buffer >> 3) & 0b1);

				ESP_LOGI("Packet", "Navigation: %d", rc.lights.isNavigationEnabled());
				ESP_LOGI("Packet", "Strobe: %d", rc.lights.isStrobeEnabled());
				ESP_LOGI("Packet", "Landing: %d", rc.lights.isLandingEnabled());
				ESP_LOGI("Packet", "Cabin: %d", rc.lights.isCabinEnabled());
				ESP_LOGI("Packet", "----------------");

				break;
			}
		}
	}
}