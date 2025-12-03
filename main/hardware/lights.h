#pragma once

#include <esp_timer.h>
#include "light.h"
#include "constants.h"

namespace pizda {
	class Lights {
		public:
			Light interior {
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

			void setup() {
				interior.fill(0x55);
				interior.flush();

//				tail.fill(0x00);
//				tail.flush();

				leftWing.fill(0x00);
				leftWing.flush();
			}

			void start() {
				xTaskCreate(taskBody, "lights", 2048, this, tskIDLE_PRIORITY, &taskHandle);
			}

			bool isNavigationEnabled() const {
				return navigationEnabled;
			}

			void setNavigationEnabled(bool value) {
				if (value == navigationEnabled)
					return;

				navigationEnabled = value;
			}

			bool isStrobeEnabled() const {
				return strobeEnabled;
			}

			void setStrobeEnabled(bool value) {
				if (value == strobeEnabled)
					return;

				strobeEnabled = value;
			}

			bool isLandingEnabled() const {
				return landingEnabled;
			}

			void setLandingEnabled(bool value) {
				if (value == landingEnabled)
					return;

				landingEnabled = value;
			}

		private:
			constexpr static const uint8_t dimmedChannelValue = 0x22;

			TaskHandle_t taskHandle;

			bool navigationEnabled = false;
			bool strobeEnabled = false;
			bool landingEnabled = false;

			void updateNavOrLanding(Light& light, const uint8_t r, const uint8_t g, const uint8_t b) const {
				// Navigation
				if (navigationEnabled) {
					light.fill(r, g, b);
				}
				else {
					light.fill(0x00);
				}

				// Landing
				if (landingEnabled)
					light.fill(0, light.getLength() / 2, 0xFF, 0xFF, 0xFF);

				light.flush();
			}

			void updateStrobes(Light& light, const uint8_t r, const uint8_t g, const uint8_t b) {
				if (strobeEnabled) {
					light.fill(0xFF);
					light.flush();
				}
				else {
					updateNavOrLanding(light, r, g, b);
				}
			}

			static void taskBody(void* args) {
				const auto lights = reinterpret_cast<Lights*>(args);

				//             0       500       1000 ms
				//             +--------+---------+
				// Left wing:  WRWRRRRRRRRRRRRRRRRR
				// Right wing: WGWGGGGGGGGGGGGGGGGG
				// Tail:       DDDWDDDDDDDDDDDDDDDD

				while (true) {
					// Left wing (strobe 1)
					lights->updateStrobes(lights->leftWing, 0xFF, 0x00, 0x00);

					// Tail (dimmed)
//					lights->tail.fill(dimmedChannelValue);
//					lights->tail.flush();

					vTaskDelay(pdMS_TO_TICKS(50));

					// Left wing (red)
					lights->updateNavOrLanding(lights->leftWing, 0xFF, 0x00, 0x00);
					vTaskDelay(pdMS_TO_TICKS(50));

					// Left wing (strobe 2)
					lights->updateStrobes(lights->leftWing, 0xFF, 0x00, 0x00);
					vTaskDelay(pdMS_TO_TICKS(50));

					// Left wing (red)
					lights->updateNavOrLanding(lights->leftWing, 0xFF, 0x00, 0x00);

					// Tail (strobe)
//					lights->tail.fill(0xFF);
//					lights->tail.flush();

					vTaskDelay(pdMS_TO_TICKS(50));

					// Tail (dimmed)
//					lights->tail.fill(dimmedChannelValue);
//					lights->tail.flush();

					vTaskDelay(pdMS_TO_TICKS(16 * 50));
				}
			}
	};
}