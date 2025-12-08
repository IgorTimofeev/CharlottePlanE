#pragma once

#include <array>
#include <optional>

#include "constants.h"
#include "hardware/motor.h"

namespace pizda {
	class Motors {
		public:
			void setup();
			void setPower(uint8_t index, uint16_t value);
			void updateFromSettings();

			std::array<std::optional<YobaMotor>, 8> motors {
				std::nullopt,
				std::nullopt,
				std::optional<YobaMotor>( Motor { constants::motors::leftAileron, LEDC_CHANNEL_0 }),
				std::nullopt,
				std::nullopt,
				std::nullopt,
				std::optional<YobaMotor>( Motor { constants::motors::leftFlap, LEDC_CHANNEL_1 }),
				std::nullopt,
			};

		private:
			void copyConfigurationsFromSettings();

	};
}