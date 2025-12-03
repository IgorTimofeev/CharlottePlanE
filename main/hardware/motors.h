#pragma once

#include "constants.h"
#include "hardware/motor.h"

namespace pizda {
	class Motors {
		public:
			void setup();

			uint16_t getLeftAileron() const;
			void setLeftAileron(uint16_t value);

			uint16_t getLeftFlap() const;
			void setLeftFlap(uint16_t value);

		private:
			uint16_t leftAileron = 0;
			uint16_t leftFlap = 0;

			Motor leftAileronMotor { constants::motors::leftAileron, LEDC_CHANNEL_0 };
			Motor leftFlapMotor { constants::motors::leftFlap, LEDC_CHANNEL_1 };
	};
}