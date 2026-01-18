#pragma once

#include <esp_timer.h>

#include "config.h"
#include "hardware/light.h"
#include "types/settings/settings.h"

namespace pizda {
	class Lights {
		public:
			void setup();
			void start();
			
			void setCabinEnabled(bool value);
			void setNavigationEnabled(bool value);
			void setStrobeEnabled(bool value);
			void setLandingEnabled(bool value);
			void setEmergencyEnabled(bool emergencyEnabled);

		private:
			constexpr static uint8_t tailDimmedValue = 0x22;
			
			TaskHandle_t taskHandle = nullptr;
			
			bool _emergency = false;
			
			Light leftWing {
				config::lights::wingLeft::pin,
				config::lights::wingLeft::length
			};
			
			Light rightWing {
				config::lights::wingRight::pin,
				config::lights::wingRight::length
			};
			
			Light tail {
				config::lights::tail::pin,
				config::lights::tail::length
			};
			
			Light cabin {
				config::lights::cabin::pin,
				config::lights::cabin::length
			};
			
			bool delay(uint32_t ms);
			void wake();
			void updateNavOrLanding(Light& light, const uint8_t r, const uint8_t g, const uint8_t b) const;
			void updateStrobes(Light& light, const uint8_t r, const uint8_t g, const uint8_t b);
			
			[[noreturn]] void onStart();
	};
}