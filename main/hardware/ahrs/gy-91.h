#pragma once

#include <driver/gpio.h>

#include "constants.h"
#include "hardware/ahrs/mpu9250.h"
#include "hardware/ahrs/bmp280.h"

namespace pizda {
	class GY91 {
		public:
			GY91(gpio_num_t mpu9250ssPin, gpio_num_t bmp280ssPin) : _mpu9520(MPU9250 { mpu9250ssPin }), _bmp280(BMP280 { bmp280ssPin }) {

			}

			void setup(spi_host_device_t SPIDevice, gpio_num_t misoPin, gpio_num_t mosiPin, gpio_num_t sckPin) {
				// GPIO
				setupGPIO();

				// MPU-9250
				_mpu9520.setup(
					SPIDevice,
					misoPin,
					mosiPin,
					sckPin,
					1'000'000
				);

				// BMP280
				_bmp280.setup(
					SPIDevice,
					misoPin,
					mosiPin,
					sckPin,
					10'000'000
				);

				_bmp280.configure(
					BMP280Mode::normal,
					BMP280Oversampling::x16,
					BMP280Oversampling::x2,
					BMP280Filter::x4,
					BMP280StandbyDuration::ms1
				);
			}

			void readPressureAndTemperature(
				float& pressurePa,
				float& temperatureC
			) {
				_bmp280.readPressureAndTemperature(pressurePa, temperatureC);
			}

		private:
			MPU9250 _mpu9520;
			BMP280 _bmp280;

			void setupGPIO() {
				gpio_config_t config {};
				config.pin_bit_mask = (1ULL << _mpu9520.getSSPin()) | (1ULL << _bmp280.getSSPin());
				config.mode = GPIO_MODE_OUTPUT;
				config.pull_up_en = GPIO_PULLUP_ENABLE;
				config.pull_down_en = GPIO_PULLDOWN_DISABLE;
				config.intr_type = GPIO_INTR_DISABLE;
				gpio_config(&config);

				gpio_set_level(_mpu9520.getSSPin(), true);
				gpio_set_level(_bmp280.getSSPin(), true);
			}
	};
}