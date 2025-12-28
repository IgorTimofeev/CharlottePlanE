#pragma once

#include <esp_timer.h>

#include "config.h"
#include "hardware/light.h"
#include "hardware/transceiver/channels.h"
#include "settings/settings.h"

namespace pizda {
	class Lights {
		public:
			void setup();
			void start();
			
			void setCabinEnabled(bool value);
			
			bool isNavigationEnabled() const;
			void setNavigationEnabled(bool value);

			bool isStrobeEnabled() const;
			void setStrobeEnabled(bool value);

			bool isLandingEnabled() const;
			void setLandingEnabled(bool value);

			bool isEmergencyEnabled() const;
			void setEmergencyEnabled(bool emergencyEnabled);

		private:
			TaskHandle_t taskHandle;

//			Strip tail {
//				config::lights::tail::pin,
//				config::lights::tail::length
//			};
			
			Light cabin {
				config::lights::cabin::pin,
				config::lights::cabin::length
			};

			Light leftWing {
				config::lights::leftWing::pin,
				config::lights::leftWing::length
			};
			
			bool cabinEnabled = false;
			bool navigationEnabled = false;
			bool strobeEnabled = false;
			bool landingEnabled = false;
			bool emergencyEnabled = false;

			void updateNavOrLanding(Light& light, const uint8_t r, const uint8_t g, const uint8_t b) const;
			void updateStrobes(Light& light, const uint8_t r, const uint8_t g, const uint8_t b);
			void taskBody();
	};
}