#pragma once

#include "constants.h"
#include "hardware/motor.h"

namespace pizda {
	class Motors {
		public:
			void setup();

			uint16_t getLeftThrottle() const;
			void setLeftThrottle(uint16_t value);

			uint16_t getRightThrottle() const;
			void setRightThrottle(uint16_t value);

			uint16_t getAilerons() const;
			void setAilerons(uint16_t value);

			uint16_t getElevator() const;
			void setElevator(uint16_t value);

			uint16_t getRudder() const;
			void setRudder(uint16_t value);

			uint16_t getFlaps() const;
			void setFlaps(uint16_t value);

		private:
			uint16_t leftThrottle = 0;
			uint16_t rightThrottle = 0;
			uint16_t ailerons = 0;
			uint16_t elevator = 0;
			uint16_t rudder = 0;
			uint16_t flaps = 0;

			Motor leftAileronMotor { constants::motors::leftAileron, LEDC_CHANNEL_0 };
			Motor leftFlapMotor { constants::motors::leftFlap, LEDC_CHANNEL_1 };
	};
}