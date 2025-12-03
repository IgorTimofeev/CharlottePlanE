#pragma once

#include "cstdint"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

namespace pizda {
	class constants {
		public:
			class i2c {
				public:
					constexpr static gpio_num_t sda = GPIO_NUM_8;
					constexpr static gpio_num_t scl = GPIO_NUM_9;
			};

			class motors {
				public:
					constexpr static gpio_num_t leftFlap = GPIO_NUM_6;
					constexpr static gpio_num_t leftAileron = GPIO_NUM_5;
			};

			class lights {
				public:
					class interior {
						public:
							// ESP-IDF is crying about missing GPIO_NUM_48, so
							constexpr static uint16_t pin = 48;
							constexpr static uint8_t length = 1;
					};

					class leftWing {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_4;
							constexpr static uint8_t length = 6;
					};

					class rightWing {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_4;
							constexpr static uint8_t length = 6;
					};

					class tail {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_4;
							constexpr static uint8_t length = 3;
					};
			};
	};
}
