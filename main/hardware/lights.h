#pragma once

#include <esp_timer.h>

#include "config.h"
#include "hardware/light.h"
#include "settings/settings.h"

namespace pizda {
	class Lights {
		public:
			void setup() const;
			void start();
			
			void setCabinEnabled(bool value) const;
			void setNavigationEnabled(bool value) const;
			void setStrobeEnabled(bool value) const;
			void setLandingEnabled(bool value) const;
			void setEmergencyEnabled(bool value);

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

			static bool delay(uint32_t ms);
			void wake() const;
			static void updateNavOrLanding(const Light& light, uint8_t r, uint8_t g, uint8_t b);
			static void updateStrobes(const Light& light, uint8_t r, uint8_t g, uint8_t b);
			
			[[noreturn]] void onStart() const;
	};
}