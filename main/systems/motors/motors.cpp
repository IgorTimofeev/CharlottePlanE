#include "systems/motors/motors.h"

#include "aircraft.h"

namespace pizda {
	void Motors::setup() {
		for (auto& pwm : _LEDCPWMs)
			pwm.setup();

		updateConfigurationsFromSettings();
	}
	
	Motor* Motors::getMotor(const uint8_t index) {
		if (index >= _motors.size()) {
			ESP_LOGI(_logTag, "index %d >= motors count %d", index, _motors.size());
			return nullptr;
		}
		
		return &_motors[index];
	}
	
	Motor* Motors::getMotor(const MotorType type) {
		return getMotor(std::to_underlying(type));
	}
	
	void Motors::updateConfigurationsFromSettings() {
		const auto& ac = Aircraft::getInstance();
		
		getMotor(MotorType::throttle)->setConfiguration(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setConfiguration(ac.settings.motors.noseWheel);
		
		getMotor(MotorType::flapLeft)->setConfiguration(ac.settings.motors.flapLeft);
		getMotor(MotorType::aileronLeft)->setConfiguration(ac.settings.motors.aileronLeft);
		
		getMotor(MotorType::flapRight)->setConfiguration(ac.settings.motors.flapRight);
		getMotor(MotorType::aileronRight)->setConfiguration(ac.settings.motors.aileronRight);
		
		getMotor(MotorType::tailLeft)->setConfiguration(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setConfiguration(ac.settings.motors.tailRight);
	}
}