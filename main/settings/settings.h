#pragma once

#include "settings/motorsSettings.h"
#include "settings/ADIRSSettings.h"
#include "settings/trimSettings.h"
#include "settings/lightsSettings.h"

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