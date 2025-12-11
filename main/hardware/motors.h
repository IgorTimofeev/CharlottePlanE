#pragma once

#include <array>
#include <optional>

#include "constants.h"
#include "hardware/motor.h"
#include <utility>

namespace pizda {
	enum class MotorType : uint8_t {
		throttle,
		reverseThrottle,

		leftAileron,
		rightAileron,

		leftTail,
		rightTail,

		leftFlap,
		rightFlap
	};

	class Motors {
		public:
			void setup();
			ConfiguredMotor* getMotor(uint8_t index);
			ConfiguredMotor* getMotor(MotorType type);
			void updateConfigurationsFromSettings();

		private:
			std::array<std::optional<ConfiguredMotor>, 8> instances {
				ConfiguredMotor {constants::motors::throttle, LEDC_CHANNEL_0 },
				ConfiguredMotor {constants::motors::throttleReverse, LEDC_CHANNEL_1 },

				ConfiguredMotor {constants::motors::leftWingAileron, LEDC_CHANNEL_2 },
				std::nullopt,

				std::nullopt,
				std::nullopt,

				ConfiguredMotor {constants::motors::leftWingFlap, LEDC_CHANNEL_3 },
				std::nullopt,
			};

	};
}