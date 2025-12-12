#pragma once

#include <cstdint>

#include <driver/gpio.h>
#include <driver/spi_master.h>

namespace pizda {
	class MPU9250 {
		public:
			void setup(spi_host_device_t SPIDevice, gpio_num_t misoPin, gpio_num_t mosiPin, gpio_num_t sckPin, gpio_num_t ssPin, uint32_t frequencyHz) {
				// GPIO
				gpio_config_t GPIOConfig {};
				GPIOConfig.pin_bit_mask = 1ULL << _ssPin;
				GPIOConfig.mode = GPIO_MODE_OUTPUT;
				GPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
				GPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
				GPIOConfig.intr_type = GPIO_INTR_DISABLE;
				gpio_config(&GPIOConfig);

				gpio_set_level(_ssPin, ssPin);

				// SPI
			}

		private:
			// SPI
			gpio_num_t _ssPin;
			spi_device_handle_t _spiDeviceHandle {};

	};
}