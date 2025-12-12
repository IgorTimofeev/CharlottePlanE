#pragma once

#include <driver/gpio.h>

#include "constants.h"
#include "hardware/ahrs/mpu9250.h"
#include "hardware/ahrs/bmp280.h"

namespace pizda {
	class GY91 {
		public:
			GY91(gpio_num_t mpu9250ssPin, gpio_num_t bmp280ssPin) :
				_mpu9520(MPU9250 {
					mpu9250ssPin
				}),
				_bmp280(BMP280 {
					bmp280ssPin
				})
			{

			}

			void setup() {
				// GPIO
				setupGPIO();

				// MPU-9250
				_mpu9520.setup(
					SPI2_HOST,
					constants::spi::miso,
					constants::spi::mosi,
					constants::spi::sck
				);

				// BMP280
				_bmp280.setup(
					SPI2_HOST,
					constants::spi::miso,
					constants::spi::mosi,
					constants::spi::sck
				);

				_bmp280.configure(
					BMP280Mode::Normal,
					BMP280Oversampling::X2,
					BMP280Oversampling::X16,
					BMP280Filter::X16,
					BMP280StandbyDuration::Ms125
				);

				xTaskCreate(
					[](void* arg) {
						reinterpret_cast<GY91*>(arg)->taskBody();
					},
					"ahrs",
					4096,
					this,
					10,
					nullptr
				);
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

			void taskBody() {
				while (true) {
					const auto pressure = _bmp280.readPressure();
					const auto temperature = _bmp280.readTemperature();

					ESP_LOGI("AHRS BMP", "Pressure: %f, temp: %f", pressure, temperature);

					vTaskDelay(pdMS_TO_TICKS(1000));
				}
			}
	};
}