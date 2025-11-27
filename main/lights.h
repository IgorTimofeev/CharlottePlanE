#pragma once

#include <esp_timer.h>
#include "led_strip_rmt.h"
#include "led_strip.h"
#include "constants.h"

namespace pizda {
	class Strip {
		public:
			Strip(uint8_t pin, uint8_t length) : length(length) {
				led_strip_config_t strip_config{};
				strip_config.strip_gpio_num = pin;
				strip_config.max_leds = length;
				strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
				strip_config.led_model = LED_MODEL_WS2812;
				strip_config.flags.invert_out = false;

				led_strip_rmt_config_t rmt_config{};
				rmt_config.clk_src = RMT_CLK_SRC_DEFAULT; // different clock source can lead to different power consumption
				rmt_config.resolution_hz = 10000000;
				rmt_config.mem_block_symbols = 0;
				rmt_config.flags.with_dma = false;

				ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &stripHandle));
			}

			uint8_t getLength() const {
				return length;
			}

			void set(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
				led_strip_set_pixel(stripHandle, index, r, g, b);
			}

			void setRange(uint8_t fromIndex, uint8_t toIndex, uint8_t r, uint8_t g, uint8_t b) {
				for (uint8_t index = fromIndex; index <= toIndex; index++) {
					set(index, r, g, b);
				}
			}

			void fill(uint8_t r, uint8_t g, uint8_t b) {
				setRange(0, length - 1, r, g, b);
			}

			void fill(uint8_t value) {
				fill(value, value, value);
			}

			void flush() {
				led_strip_refresh(stripHandle);
			}

		private:
			uint8_t length;
			led_strip_handle_t stripHandle{};
	};

	class Lights {
		public:
			Strip interior {
				constants::LEDStrip::interior::pin,
				constants::LEDStrip::interior::length
			};

			Strip tail {
				constants::LEDStrip::tail::pin,
				constants::LEDStrip::tail::length
			};

			Strip leftWing {
				constants::LEDStrip::tail::pin,
				constants::LEDStrip::tail::length
			};

			void setup() {
				interior.fill(0x22);
				interior.flush();

//				TaskHandle_t tailTaskHandle;
//				xTaskCreate(tailTask, "tailLights", 1024, &tail, tskIDLE_PRIORITY, &tailTaskHandle);

				TaskHandle_t leftWingHandle;
				xTaskCreate(leftWingTask, "leftWingLights", 1024, &leftWing, tskIDLE_PRIORITY, &leftWingHandle);
			}

		private:
			static void interiorTask(void* args) {
				auto strip = reinterpret_cast<Strip*>(args);

				while (true) {
					strip->fill(0x11);
					strip->flush();
					vTaskDelay(pdMS_TO_TICKS(1000));

					strip->fill(0x00);
					strip->flush();
					vTaskDelay(pdMS_TO_TICKS(1000));
				}
			}

			static void tailTask(void* args) {
				auto strip = reinterpret_cast<Strip*>(args);

				while (true) {
					strip->fill(0xFF, 0xFF, 0xFF);
					strip->flush();
					vTaskDelay(pdMS_TO_TICKS(100));

					strip->fill(0x22, 0x22, 0x22);
					strip->flush();
					vTaskDelay(pdMS_TO_TICKS(900));
				}
			}

			static void wingTick(Strip* strip, uint8_t r, uint8_t g, uint8_t b) {
				strip->fill(0xFF, 0xFF, 0xFF);
				strip->flush();
				vTaskDelay(pdMS_TO_TICKS(50));

				strip->fill(r, g, b);
				strip->flush();
				vTaskDelay(pdMS_TO_TICKS(50));

				strip->fill(0xFF, 0xFF, 0xFF);
				strip->flush();
				vTaskDelay(pdMS_TO_TICKS(50));

				strip->fill(r, g, b);
				strip->flush();
				vTaskDelay(pdMS_TO_TICKS(850));
			}

			static void leftWingTask(void* args) {
				while (true) {
					wingTick(reinterpret_cast<Strip*>(args), 0xFF, 0x00, 0x00);
				}
			}
	};
}