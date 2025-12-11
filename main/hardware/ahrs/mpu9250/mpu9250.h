#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace pizda {
	class MPU9250 {
		public:
			MPU9250(gpio_num_t misoPin, gpio_num_t mosiPin, gpio_num_t sckPin, gpio_num_t ssPin) : _misoPin(misoPin), _mosiPin(mosiPin), _sckPin(sckPin), _ssPin(ssPin) {

			}

			void setup(

			) {
				setupGPIO();
				gpio_set_level(_ssPin, true);
			}

		private:
			// SPI
			gpio_num_t _misoPin;
			gpio_num_t _mosiPin;
			gpio_num_t _sckPin;
			gpio_num_t _ssPin;

			spi_device_handle_t _spiDeviceHandle {};

			void setupGPIO() {
				gpio_config_t config {};
				config.pin_bit_mask = 1ULL << _ssPin;
				config.mode = GPIO_MODE_OUTPUT;
				config.pull_up_en = GPIO_PULLUP_ENABLE;
				config.pull_down_en = GPIO_PULLDOWN_DISABLE;
				config.intr_type = GPIO_INTR_DISABLE;
				gpio_config(&config);
			}

	};
}