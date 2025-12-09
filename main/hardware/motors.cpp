#include "motors.h"

#include "hardware/channels.h"
#include "rc.h"

namespace pizda {
	void Motors::setup() {

		auto& rc = RC::getInstance();

		for (size_t i = 0; i < rc.settings.motors.configurations.size(); ++i) {
			if (i < instances.size() && instances[i]) {
				instances[i]->configuration = rc.settings.motors.configurations[i];
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
		auto& rc = RC::getInstance();

		for (size_t i = 0; i < rc.settings.motors.configurations.size(); ++i) {
			if (i < instances.size() && instances[i]) {
				instances[i]->configuration = rc.settings.motors.configurations[i];
				instances[i]->updatePowerFromConfiguration();
			}
		}
	}
}