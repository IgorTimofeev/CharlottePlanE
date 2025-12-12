#pragma once

#include <cstdint>

#include <driver/gpio.h>
#include <driver/spi_master.h>

namespace pizda {
	class MPU9250 {
		public:
			MPU9250(gpio_num_t ssPin) :_ssPin(ssPin) {

			}

			void setup(spi_host_device_t spiDevice, gpio_num_t misoPin, gpio_num_t mosiPin, gpio_num_t sckPin) {
				setupGPIO();
				setSlaveSelect(true);
			}

			void setSlaveSelect(bool value) {
				gpio_set_level(_ssPin, value);
			}

			gpio_num_t getSSPin() const {
				return _ssPin;
			}

		private:
			// SPI
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