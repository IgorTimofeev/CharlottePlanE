#pragma once

#include "types/settings/motorSettings.h"
#include "types/settings/ADIRSSettings.h"

namespace pizda {
	class Settings {
		public:
			MotorSettings motors {};
			ADIRSSettings adirs {};

			void setup() {
				motors.read();
				adirs.read();
			}
	};
}