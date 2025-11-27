#pragma once

#include <esp_timer.h>
#include "strip.h"
#include "config.h"

namespace pizda {
	class Lights {
		public:
//			Strip interior {
//				config::lights::interior::pin,
//				config::lights::interior::length
//			};

			Strip tail {
				config::lights::tail::pin,
				config::lights::tail::length
			};

			Strip leftWing {
				config::lights::leftWing::pin,
				config::lights::leftWing::length
			};

			void setup() {
//				interior.fill(0xFF);
//				interior.flush();

				TaskHandle_t taskHandle;
				xTaskCreate(taskBody, "lights", 1024, this, tskIDLE_PRIORITY, &taskHandle);
			}

		private:
			constexpr static const uint8_t tailDimmedValue = 0x22;

			static void taskBody(void* args) {
				const auto lights = static_cast<Lights*>(args);

				//             0       500       1000 ms
				//             +--------+---------+
				// Left wing:  WRWRRRRRRRRRRRRRRRRR
				// Right wing: WGWGGGGGGGGGGGGGGGGG
				// Tail:       DDDDDDDDDWWDDDDDDDDD

				while (true) {
					// Left wing (short strobe 1)
					lights->leftWing.fill(0xFF);
					lights->leftWing.flush();

					// Tail (dimmed)
					lights->tail.fill(tailDimmedValue);
					lights->tail.flush();

					vTaskDelay(pdMS_TO_TICKS(50));

					// Left wing (red)
					lights->leftWing.fill(0xFF, 0x00, 0x00);
					lights->leftWing.flush();

					vTaskDelay(pdMS_TO_TICKS(50));

					// Left wing (short strobe 2)
					lights->leftWing.fill(0xFF);
					lights->leftWing.flush();

					vTaskDelay(pdMS_TO_TICKS(50));

					// Left wing (red)
					lights->leftWing.fill(0xFF, 0x00, 0x00);
					lights->leftWing.flush();

					vTaskDelay(pdMS_TO_TICKS(6 * 50));

					// Tail (long strobe)
					lights->tail.fill(0xFF);
					lights->tail.flush();

					vTaskDelay(pdMS_TO_TICKS(100));

					// Tail (dimmed)
					lights->tail.fill(tailDimmedValue);
					lights->tail.flush();

					vTaskDelay(pdMS_TO_TICKS(9 * 50));
				}
			}
	};
}