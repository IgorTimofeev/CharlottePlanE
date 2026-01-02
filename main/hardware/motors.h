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
		rightFlap,
		
		noseWheel
	};

	class Motors {
		public:
			void setup();
			ConfiguredMotor* getMotor(uint8_t index);
			ConfiguredMotor* getMotor(MotorType type);
			void updateConfigurationsFromSettings();

		private:
			constexpr static const char* _logTag = "Motors";

			std::array<ConfiguredMotor, 8> _instances {
				ConfiguredMotor { config::motors::throttle, LEDC_CHANNEL_0 },

				ConfiguredMotor { config::motors::leftAileron, LEDC_CHANNEL_1 },
				ConfiguredMotor { config::motors::rightAileron, LEDC_CHANNEL_2 },
				
				ConfiguredMotor { config::motors::tailLeft, LEDC_CHANNEL_3 },
				ConfiguredMotor { config::motors::tailRight, LEDC_CHANNEL_4 },
				
				ConfiguredMotor { config::motors::leftFlap, LEDC_CHANNEL_5 },
				ConfiguredMotor { config::motors::rightFlap, LEDC_CHANNEL_6 },
				
				ConfiguredMotor { config::motors::noseWheel, LEDC_CHANNEL_7 },
			};
	};
}