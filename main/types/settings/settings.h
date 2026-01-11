#pragma once

#include "types/settings/motorSettings.h"
#include "types/settings/ADIRSSettings.h"
#include "types/settings/controlSettings.h"

namespace pizda {
	class Settings {
		public:
			MotorSettings motors {};
			ADIRSSettings adirs {};
			ControlSettings controls {};

			void setup() {
				motors.read();
				adirs.read();
			}
	};
}