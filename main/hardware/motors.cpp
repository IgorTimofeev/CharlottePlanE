#include "motors.h"

#include "hardware/transceiver/channels.h"
#include "aircraft.h"

namespace pizda {
	void Motors::setup() {
		auto& ac = Aircraft::getInstance();

		// Copying configuration from settings
		for (size_t i = 0; i < ac.settings.motors.configurations.size(); ++i) {
			const auto motor = getMotor(i);

			if (motor) {
				motor->setConfiguration(ac.settings.motors.configurations[i]);
				motor->setup();
			}
		}
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

		for (size_t i = 0; i < ac.settings.motors.configurations.size(); ++i) {
			const auto motor = getMotor(i);

			if (motor) {
				motor->setConfiguration(ac.settings.motors.configurations[i]);
				motor->updateCurrentPowerFromConfiguration();
			}
		}
	}
}