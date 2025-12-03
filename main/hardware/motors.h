#pragma once

#include "config.h"
#include "hardware/motor.h"

namespace pizda {
	class Motors {
		public:
			Motor leftFlap {
				config::servos::leftWingFlap,
				LEDC_CHANNEL_0,
				1000,
				2000
			};

			Motor leftAileron {
				config::servos::leftWingAileron,
				LEDC_CHANNEL_1,
				1000,
				2000
			};

			void setup() {
				leftFlap.setup(leftFlap.getPulseWidthFromPercent(50));
				leftAileron.setup(leftFlap.getPulseWidthFromPercent(50));
			}

		private:
	};
}