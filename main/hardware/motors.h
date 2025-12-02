#pragma once

#include "config.h"
#include "hardware/motor.h"

namespace pizda {
	class Motors {
		public:
			Motor leftWingFlap {
				config::servos::leftWingFlap,
				LEDC_CHANNEL_0,
				1000,
				2000
			};

			Motor leftWingAileron {
				config::servos::leftWingAileron,
				LEDC_CHANNEL_1,
				1000,
				2000
			};

			void setup() {
				leftWingFlap.setup(leftWingFlap.getPulseWidthFromPercent(50));
				leftWingAileron.setup(leftWingFlap.getPulseWidthFromPercent(50));
			}

		private:
	};
}