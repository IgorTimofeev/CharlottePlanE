#pragma once

#include "types/settings/motorsSettings.h"
#include "types/settings/ADIRSSettings.h"
#include "types/settings/trimSettings.h"
#include "types/settings/lightsSettings.h"

namespace pizda {
	class Settings {
		public:
			MotorsSettings motors {};
			ADIRSSettings adirs {};
			LightsSettings lights {};
			TrimSettings trim {};

			void setup() {
				motors.read();
				trim.read();
				lights.read();
				adirs.read();
			}
	};
}