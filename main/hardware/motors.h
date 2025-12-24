#pragma once

#include <array>
#include <optional>

#include "config.h"
#include "hardware/motor.h"
#include <utility>

namespace pizda {
	enum class MotorType : uint8_t {
		throttle,

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
			constexpr static const char* _logTag = "Motors";

			std::array<std::optional<ConfiguredMotor>, 7> instances {
				ConfiguredMotor { config::motors::throttle, LEDC_CHANNEL_0 },

				ConfiguredMotor { config::motors::leftWingAileron, LEDC_CHANNEL_2 },
				std::nullopt,

				std::nullopt,
				std::nullopt,

				ConfiguredMotor { config::motors::leftWingFlap, LEDC_CHANNEL_3 },
				std::nullopt,
			};

	};
}