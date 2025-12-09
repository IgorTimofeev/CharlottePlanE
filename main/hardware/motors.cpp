#include "motors.h"

#include "hardware/transceiver/channels.h"
#include "aircraft.h"

namespace pizda {
	void Motors::setup() {
		auto& ac = Aircraft::getInstance();

		// Copying configuration from settings
		for (size_t i = 0; i < ac.settings.motors.configurations.size(); ++i) {
			const auto motor = getMotor(i);

			if (!motor)
				break;

			motor->setConfiguration(ac.settings.motors.configurations[i]);
			motor->setup();
		}
	}

	ConfiguredMotor* Motors::getMotor(uint8_t index) {
		if (index >= instances.size()) {
			ESP_LOGI("Motors::setPower()", "index %d >= motors count %d", index, instances.size());
			return nullptr;
		}
		else if (!instances[index]) {
			ESP_LOGI("Motors::setPower()", "motor with index %d is not bound");
			return nullptr;
		}

		return &instances[index].value();
	}

	void Motors::updateConfigurationsFromSettings() {
		auto& ac = Aircraft::getInstance();

		for (size_t i = 0; i < ac.settings.motors.configurations.size(); ++i) {
			const auto motor = getMotor(i);

			if (!motor)
				break;

			motor->setConfiguration(ac.settings.motors.configurations[i]);
			motor->updateCurrentPowerFromConfiguration();
		}
	}
}