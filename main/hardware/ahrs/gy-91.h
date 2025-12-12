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
					BMP280Mode::normal,
					BMP280Oversampling::x16,
					BMP280Oversampling::x2,
					BMP280Filter::x8,
					BMP280StandbyDuration::ms125
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

			float getAltitude(
				float pressurePa,
				float temperatureC,
				float referencePressurePa = 101325.0f,
				float lapseRateKpm = -0.0065f
			) {
				// Physical constants
				constexpr static float g = 9.80665f;       // Gravitational acceleration (m/s²)
				constexpr static float R = 8.314462618f;   // Universal gas constant (J/(mol·K))
				constexpr static float M = 0.0289644f;     // Molar mass of dry air (kg/mol)

				// Convert temperature from Celsius to Kelvin
				const float temperatureK = temperatureC + 273.15f;

				// Avoid division by zero and invalid values
				if (pressurePa <= 0.0f || referencePressurePa <= 0.0f || temperatureK <= 0.0f)
					return 0.0f;

				// Barometric formula with temperature gradient consideration
				// Using International Standard Atmosphere (ISA) model
				// h = (T0 / L) * (1 - (P / P0)^(R * L / (g * M)))

				// If temperature lapse rate is close to zero, use simplified formula
				if (std::abs(lapseRateKpm) < 1e-6f) {
					// Isothermal atmosphere (lapse rate ≈ 0)
					return (R * temperatureK) / (g * M) * std::log(referencePressurePa / pressurePa);
				}

				// Full formula with temperature gradient
				const float exponent = (R * lapseRateKpm) / (g * M);
				const float power = std::pow(pressurePa / referencePressurePa, exponent);
				const float altitude = (temperatureK / lapseRateKpm) * (1.0f - power);

				return altitude;
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
					const auto altitude = getAltitude(pressure, temperature);

					ESP_LOGI("AHRS BMP", "Pressure: %f, temp: %f, alt: %f", pressure, temperature, altitude);

					vTaskDelay(pdMS_TO_TICKS(1000));
				}
			}
	};
}