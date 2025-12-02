#pragma once

#include <esp_timer.h>
#include "strip.h"
#include "config.h"

namespace pizda {
	class Lights {
		public:
			Strip interior {
				config::lights::interior::pin,
				config::lights::interior::length
			};

//			Strip tail {
//				config::lights::tail::pin,
//				config::lights::tail::length
//			};

			Strip leftWing {
				config::lights::leftWing::pin,
				config::lights::leftWing::length
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
				TaskHandle_t taskHandle;
				xTaskCreate(taskBody, "lights", 2048, this, tskIDLE_PRIORITY, &taskHandle);
			}

			bool isNavigationEnabled() const {
				return navigationEnabled;
			}

			void setNavigationEnabled(bool value) {
				navigationEnabled = value;
			}

			bool isStrobeEnabled() const {
				return strobeEnabled;
			}

			void setStrobeEnabled(bool value) {
				strobeEnabled = value;
			}

			bool isLandingEnabled() const {
				return landingEnabled;
			}

			void setLandingEnabled(bool value) {
				landingEnabled = value;
			}

		private:
			constexpr static const uint8_t dimmedChannelValue = 0x22;

			bool navigationEnabled = false;
			bool strobeEnabled = false;
			bool landingEnabled = false;

			void updateNavOrLanding(Strip& strip, const uint8_t r, const uint8_t g, const uint8_t b) {
				// Navigation
				if (navigationEnabled) {
					strip.fill(r, g, b);
				}
				else {
					strip.fill(0x00);
				}

				// Landing
				if (landingEnabled)
					strip.fill(0, strip.getLength() / 2, 0xFF, 0xFF, 0xFF);

				strip.flush();
			}

			void updateStrobes(Strip& strip, const uint8_t r, const uint8_t g, const uint8_t b) {
				if (strobeEnabled) {
					strip.fill(0xFF);
					strip.flush();
				}
				else {
					updateNavOrLanding(strip, r, g, b);
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