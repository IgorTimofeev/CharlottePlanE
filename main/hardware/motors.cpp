#include "motors.h"

#include "aircraft.h"

namespace pizda {
	void Motors::setup() {
		auto& ac = Aircraft::getInstance();
		
		getMotor(MotorType::throttle)->setup(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setup(ac.settings.motors.noseWheel);
		getMotor(MotorType::aileronLeft)->setup(ac.settings.motors.aileronLeft);
		getMotor(MotorType::aileronRight)->setup(ac.settings.motors.aileronRight);
		getMotor(MotorType::flapLeft)->setup(ac.settings.motors.flapRight);
		getMotor(MotorType::flapRight)->setup(ac.settings.motors.flapLeft);
		getMotor(MotorType::tailLeft)->setup(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setup(ac.settings.motors.tailRight);
	}

	ConfiguredMotor* Motors::getMotor(uint8_t index) {
		if (index >= _instances.size()) {
			ESP_LOGI(_logTag, "index %d >= motors count %d", index, _instances.size());
			return nullptr;
		}

		return &_instances[index];
	}

	ConfiguredMotor* Motors::getMotor(MotorType type) {
		return getMotor(std::to_underlying(type));
	}

	void Motors::updateConfigurationsFromSettings() {
		auto& ac = Aircraft::getInstance();
		
		getMotor(MotorType::throttle)->setConfiguration(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setConfiguration(ac.settings.motors.noseWheel);
		getMotor(MotorType::aileronLeft)->setConfiguration(ac.settings.motors.aileronLeft);
		getMotor(MotorType::aileronRight)->setConfiguration(ac.settings.motors.aileronRight);
		getMotor(MotorType::flapLeft)->setConfiguration(ac.settings.motors.flapRight);
		getMotor(MotorType::flapRight)->setConfiguration(ac.settings.motors.flapLeft);
		getMotor(MotorType::tailLeft)->setConfiguration(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setConfiguration(ac.settings.motors.tailRight);
	}
}