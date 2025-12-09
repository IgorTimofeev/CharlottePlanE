#include "motors.h"

#include "hardware/transceiver/channels.h"
#include "aircraft.h"

namespace pizda {
	void Motors::setup() {
		auto& ac = Aircraft::getInstance();

		for (size_t i = 0; i < ac.settings.motors.configurations.size(); ++i) {
			if (i < instances.size() && instances[i]) {
				instances[i]->configuration = ac.settings.motors.configurations[i];
				instances[i]->setup();
			}
		}
	}

	void Motors::setPower(uint8_t index, uint16_t value) {
		if (index >= instances.size()) {
			ESP_LOGI("Motors::setPower()", "index %d >= motors count %d", index, instances.size());
			return;
		}
		else if (!instances[index]) {
			ESP_LOGI("Motors::setPower()", "motor with index %d is not bound");
			return;
		}

		instances[index]->setPower(value);
	}

	void Motors::updateFromSettings() {
		auto& ac = Aircraft::getInstance();

		for (size_t i = 0; i < ac.settings.motors.configurations.size(); ++i) {
			if (i < instances.size() && instances[i]) {
				instances[i]->configuration = ac.settings.motors.configurations[i];
				instances[i]->updatePowerFromConfiguration();
			}
		}
	}
}