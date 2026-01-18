#pragma once

#include "types/settings/motorsSettings.h"
#include "types/settings/ADIRSSettings.h"
#include "types/settings/controlSettings.h"
#include "types/settings/lightsSettings.h"

namespace pizda {
	class Settings {
		public:
			MotorsSettings motors {};
			ADIRSSettings adirs {};
			LightsSettings lights {};
			ControlSettings controls {};

			void setup() {
				motors.read();
				controls.read();
				lights.read();
				adirs.read();
			}
	};
}