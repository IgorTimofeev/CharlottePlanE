#include "motors.h"

#include "hardware/channels.h"
#include "rc.h"

namespace pizda {
	void Motors::setup() {
		copyConfigurationsFromSettings();

		for (auto& motor : motors) {
			if (motor) {
				motor->setup();
			}
		}
	}

	void Motors::setPower(uint8_t index, uint16_t value) {
		if (index >= motors.size()) {
			ESP_LOGI("Motors::setPower()", "index %d >= motors count %d", index, motors.size());
			return;
		}
		else if (!motors[index]) {
			ESP_LOGI("Motors::setPower()", "motor with index %d is not bound");
			return;
		}

		motors[index]->setPower(value);
	}

	void Motors::copyConfigurationsFromSettings() {
		auto& rc = RC::getInstance();

		for (size_t i = 0; i < rc.settings.motors.configurations.size(); ++i) {
			if (i < motors.size()) {
				if (motors[i]) {
					motors[i]->configuration = rc.settings.motors.configurations[i];
				}
			}
		}
	}

	void Motors::updateFromSettings() {
		copyConfigurationsFromSettings();

		for (auto& motor : motors) {
			if (motor) {
				motor->setPower(motor->getPower());
			}
		}
	}
}