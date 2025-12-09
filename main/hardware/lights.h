#pragma once

#include <esp_timer.h>
#include "light.h"
#include "constants.h"
#include "channels.h"
#include "settings/settings.h"

namespace pizda {
	class Lights {
		public:
			void setup();

			void start();

			bool isNavigationEnabled() const;

			void setNavigationEnabled(bool value);

			bool isStrobeEnabled() const;

			void setStrobeEnabled(bool value);

			bool isLandingEnabled() const;

			void setLandingEnabled(bool value);

			bool isCabinEnabled() const;

			void setCabinEnabled(bool value);

			void setPower(uint8_t index, uint16_t value);

		private:
			constexpr static const uint8_t dimmedChannelValue = 0x22;

			TaskHandle_t taskHandle;

			Light cabin {
				constants::lights::interior::pin,
				constants::lights::interior::length
			};

//			Strip tail {
//				config::lights::tail::pin,
//				config::lights::tail::length
//			};

			Light leftWing {
				constants::lights::leftWing::pin,
				constants::lights::leftWing::length
			};

			bool navigationEnabled = false;
			bool strobeEnabled = false;
			bool landingEnabled = false;
			bool cabinEnabled = false;

			void updateNavOrLanding(Light& light, const uint8_t r, const uint8_t g, const uint8_t b) const;

			void updateStrobes(Light& light, const uint8_t r, const uint8_t g, const uint8_t b);

			static void taskBody(void* args);
	};
}