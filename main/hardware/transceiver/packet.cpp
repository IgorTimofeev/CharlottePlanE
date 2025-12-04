#include "packet.h"

#include <cmath>

#include <YOBABitStream/main.h>

#include "settings.h"
#include "hardware/motor.h"
#include "rc.h"
#include <span>

namespace pizda {
	uint8_t Packet::getCRC8(const uint8_t* buffer, size_t length) {
		uint8_t crc = 0xff;
		size_t i, j;

		for (i = 0; i < length; i++) {
			crc ^= buffer[i];

			for (j = 0; j < 8; j++) {
				if ((crc & 0x80) != 0)
					crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
				else
					crc <<= 1;
			}
		}

		return crc;
	}

	bool Packet::checkCRC(const uint8_t* buffer, size_t length) {
		const auto checksum = getCRC8(buffer, length);
		const auto expectedChecksum = *(buffer + length);

		if (checksum != expectedChecksum) {
			ESP_LOGE("Packet", "Checksum mismatch: got %d, expected %d", checksum, expectedChecksum);

			return false;
		}

		return true;
	}

	bool Packet::readValueCountAndCheckCRC(
		const uint8_t* crcStart,
		ReadableBitStream& bitStream,
		uint8_t valueBitCount,
		uint8_t valueCountBitCount,
		uint8_t valueCountExpected,
		uint8_t* valueCount
	) {
		*valueCount = bitStream.readUint8(valueCountBitCount);

		ESP_LOGI("Packet", "Value count: %d", *valueCount);

		if (*valueCount != valueCountExpected) {
			ESP_LOGE("Packet", "Value count mismatch: got %d, expected %d", *valueCount, valueCountExpected);
			return false;
		}

		const auto dataBitCount = packetTypeBitCount + valueCountBitCount + valueBitCount * *valueCount;
		const uint8_t dataByteCount = (dataBitCount + 7) / 8;

		return checkCRC(crcStart, dataByteCount);
	}

	void Packet::parse(uint8_t* buffer) {
		auto& rc = RC::getInstance();

		ESP_LOGI("Packet", "----------------");

		// Header
		if (memcmp(buffer, Packet::header, Packet::headerByteCount) != 0) {
			ESP_LOGE("Packet", "Mismatched header: %s", buffer);

			return;
		}

		buffer += Packet::headerByteCount;

		// Type
		ReadableBitStream bitStream { buffer };

		const auto packetType = static_cast<PacketType>(bitStream.readUint16(4));

		ESP_LOGI("Packet", "Type: %d", static_cast<uint8_t>(packetType));

		switch (packetType) {
			case PacketType::RemoteSetAnalogValues: {
				uint8_t valueCount = 0;

				if (!readValueCountAndCheckCRC(
					buffer,
					bitStream,
					Motor::powerBitCount,
					4,
					5,
					&valueCount
				))
					return;

				rc.motors.setThrottle(bitStream.readUint16(Motor::powerBitCount));
				rc.motors.setAilerons(bitStream.readUint16(Motor::powerBitCount));
				rc.motors.setElevator(bitStream.readUint16(Motor::powerBitCount));
				rc.motors.setRudder(bitStream.readUint16(Motor::powerBitCount));
				rc.motors.setFlaps(bitStream.readUint16(Motor::powerBitCount));

				ESP_LOGI("Packet", "Throttle: %d", rc.motors.getThrottle());
				ESP_LOGI("Packet", "Ailerons: %d", rc.motors.getAilerons());
				ESP_LOGI("Packet", "Elevator: %d", rc.motors.getElevator());
				ESP_LOGI("Packet", "Rudder: %d", rc.motors.getRudder());
				ESP_LOGI("Packet", "Flaps: %d", rc.motors.getFlaps());

				break;
			}
			case PacketType::RemoteSetMotorConfigurations: {
				uint8_t valueCount = 0;

				if (!readValueCountAndCheckCRC(
					buffer,
					bitStream,
					Motor::powerBitCount + Motor::powerBitCount + Motor::powerBitCount + 1,
					4,
					8,
					&valueCount
				))
					return;

				const auto readSettings = [&bitStream](MotorSettings& settings) {
					settings.min = bitStream.readUint16(Motor::powerBitCount);
					settings.max = bitStream.readUint16(Motor::powerBitCount);
					settings.offset = bitStream.readInt16(Motor::powerBitCount);
					settings.reverse = bitStream.readBool();
					settings.sanitize();
				};

				readSettings(rc.settings.motors.leftThrottle);
				readSettings(rc.settings.motors.rightThrottle);

				readSettings(rc.settings.motors.leftAileron);
				readSettings(rc.settings.motors.rightAileron);

				readSettings(rc.settings.motors.leftTail);
				readSettings(rc.settings.motors.rightTail);

				readSettings(rc.settings.motors.leftFlap);
				readSettings(rc.settings.motors.rightFlap);

				rc.settings.motors.scheduleWrite();

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

				break;
			}
			case PacketType::RemoteSetBooleanValues: {
				uint8_t valueCount = 0;

				if (!readValueCountAndCheckCRC(
					buffer,
					bitStream,
					1,
					4,
					4,
					&valueCount
				))
					return;

				rc.lights.setNavigationEnabled(bitStream.readBool());
				rc.lights.setStrobeEnabled(bitStream.readBool());
				rc.lights.setLandingEnabled(bitStream.readBool());
				rc.lights.setCabinEnabled(bitStream.readBool());

				ESP_LOGI("Packet", "Navigation: %d", rc.lights.isNavigationEnabled());
				ESP_LOGI("Packet", "Strobe: %d", rc.lights.isStrobeEnabled());
				ESP_LOGI("Packet", "Landing: %d", rc.lights.isLandingEnabled());
				ESP_LOGI("Packet", "Cabin: %d", rc.lights.isCabinEnabled());

				break;
			}
			default: {
				ESP_LOGE("Packet", "Unknown packet type: %d", static_cast<uint8_t>(packetType));

				break;
			}
		}

		// Total length
		ESP_LOGI("Packet", "Total length: %d", headerByteCount + bitStream.getBytesRead() + 1);
		ESP_LOGI("Packet", "----------------");
	}
}