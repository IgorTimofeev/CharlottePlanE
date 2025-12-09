#pragma once

#include <array>
#include <optional>

#include "constants.h"
#include "hardware/motor.h"

namespace pizda {
	class Motors {
		public:
			void setup();
			ConfiguredMotor* getMotor(uint8_t index);
			void updateConfigurationsFromSettings();

		private:
			std::array<std::optional<ConfiguredMotor>, 8> instances {
				std::nullopt,
				std::nullopt,
				ConfiguredMotor {constants::motors::leftAileron, LEDC_CHANNEL_0 },
				std::nullopt,
				std::nullopt,
				std::nullopt,
				ConfiguredMotor {constants::motors::leftFlap, LEDC_CHANNEL_1 },
				std::nullopt,
			};

	};
}