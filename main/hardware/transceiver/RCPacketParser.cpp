#include "RCPacketParser.h"

#include "rc.h"
#include "hardware/motor.h"

namespace pizda {
	bool RCPacketParser::onParse(ReadableBitStream& stream, PacketType packetType) {
		auto& rc = RC::getInstance();

		switch (packetType) {
			case PacketType::RemoteSetAnalogValues: {
				const auto valueCount = readValueCountAndCheckCRC(stream, 5, 4, Motor::powerBitCount);

				if (!valueCount)
					return false;

				rc.motors.setThrottle(stream.readUint16(Motor::powerBitCount));
				rc.motors.setAilerons(stream.readUint16(Motor::powerBitCount));
				rc.motors.setElevator(stream.readUint16(Motor::powerBitCount));
				rc.motors.setRudder(stream.readUint16(Motor::powerBitCount));
				rc.motors.setFlaps(stream.readUint16(Motor::powerBitCount));

				ESP_LOGI("RCPacketParser", "throttle: %d", rc.motors.getThrottle());
				ESP_LOGI("RCPacketParser", "ailerons: %d", rc.motors.getAilerons());
				ESP_LOGI("RCPacketParser", "elevator: %d", rc.motors.getElevator());
				ESP_LOGI("RCPacketParser", "rudder: %d", rc.motors.getRudder());
				ESP_LOGI("RCPacketParser", "flaps: %d", rc.motors.getFlaps());

				break;
			}
			case PacketType::RemoteSetMotorConfigurations: {
				const auto valueCount = readValueCountAndCheckCRC(
					stream,
					8,
					4,
					Motor::powerBitCount + Motor::powerBitCount + Motor::powerBitCount + 1
				);

				if (!valueCount)
					return false;

				const auto readIntoSettings = [&stream](MotorSettings& settings) {
					settings.min = stream.readUint16(Motor::powerBitCount);
					settings.max = stream.readUint16(Motor::powerBitCount);
					settings.offset = stream.readInt16(Motor::powerBitCount);
					settings.reverse = stream.readBool();
				};

				readIntoSettings(rc.settings.motors.leftThrottle);
				readIntoSettings(rc.settings.motors.rightThrottle);

				readIntoSettings(rc.settings.motors.leftAileron);
				readIntoSettings(rc.settings.motors.rightAileron);

				readIntoSettings(rc.settings.motors.leftTail);
				readIntoSettings(rc.settings.motors.rightTail);

				readIntoSettings(rc.settings.motors.leftFlap);
				readIntoSettings(rc.settings.motors.rightFlap);

				rc.settings.motors.scheduleWrite();

				// Updating motors position
				rc.motors.setAilerons(rc.motors.getAilerons());
				rc.motors.setFlaps(rc.motors.getFlaps());

				ESP_LOGI("RCPacketParser", "left throttle min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftThrottle.min, rc.settings.motors.leftThrottle.max, rc.settings.motors.leftThrottle.offset, rc.settings.motors.leftThrottle.reverse);
				ESP_LOGI("RCPacketParser", "right throttle min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightThrottle.min, rc.settings.motors.rightThrottle.max, rc.settings.motors.rightThrottle.offset, rc.settings.motors.rightThrottle.reverse);
				ESP_LOGI("RCPacketParser", "left aileron min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftAileron.min, rc.settings.motors.leftAileron.max, rc.settings.motors.leftAileron.offset, rc.settings.motors.leftAileron.reverse);
				ESP_LOGI("RCPacketParser", "right aileron min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightAileron.min, rc.settings.motors.rightAileron.max, rc.settings.motors.rightAileron.offset, rc.settings.motors.rightAileron.reverse);
				ESP_LOGI("RCPacketParser", "left tail min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftTail.min, rc.settings.motors.leftTail.max, rc.settings.motors.leftTail.offset, rc.settings.motors.leftTail.reverse);
				ESP_LOGI("RCPacketParser", "right tail min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightTail.min, rc.settings.motors.rightTail.max, rc.settings.motors.rightTail.offset, rc.settings.motors.rightTail.reverse);
				ESP_LOGI("RCPacketParser", "left flap min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.leftFlap.min, rc.settings.motors.leftFlap.max, rc.settings.motors.leftFlap.offset, rc.settings.motors.leftFlap.reverse);
				ESP_LOGI("RCPacketParser", "right flap min: %d, max: %d, offset: %d, reverse: %d", rc.settings.motors.rightFlap.min, rc.settings.motors.rightFlap.max, rc.settings.motors.rightFlap.offset, rc.settings.motors.rightFlap.reverse);

				break;
			}
			case PacketType::RemoteSetBooleanValues: {
				const auto valueCount = readValueCountAndCheckCRC(stream, 4, 4, 1);

				if (!valueCount)
					return false;

				rc.lights.setNavigationEnabled(stream.readBool());
				rc.lights.setStrobeEnabled(stream.readBool());
				rc.lights.setLandingEnabled(stream.readBool());
				rc.lights.setCabinEnabled(stream.readBool());

				ESP_LOGI("RCPacketParser", "navigation: %d", rc.lights.isNavigationEnabled());
				ESP_LOGI("RCPacketParser", "strobe: %d", rc.lights.isStrobeEnabled());
				ESP_LOGI("RCPacketParser", "landing: %d", rc.lights.isLandingEnabled());
				ESP_LOGI("RCPacketParser", "cabin: %d", rc.lights.isCabinEnabled());

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