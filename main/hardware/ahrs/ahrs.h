#pragma once

#include <array>

#include <driver/spi_master.h>
#include <driver/gpio.h>

#include "constants.h"
#include "hardware/ahrs/mpu9250.h"
#include "hardware/ahrs/bmp280.h"

namespace pizda {
	template<typename T>
	class AHRSUnitAndSSPin {
		public:
			explicit AHRSUnitAndSSPin(gpio_num_t ssPin) : pin(ssPin) {

			}

			T unit {};
			gpio_num_t pin;
	};

	class AHRS {
		public:
			void setup() {
				// Setting every CS to high just in case
				uint64_t SSPinBitMask = 0;

				for (auto& unitAndPin : _MPUs)
					SSPinBitMask |= (1ULL << unitAndPin.pin);

				for (auto& unitAndPin : _BMPs)
					SSPinBitMask |= (1ULL << unitAndPin.pin);

				gpio_config_t GPIOConfig {};
				GPIOConfig.pin_bit_mask = 1ULL << SSPinBitMask;
				GPIOConfig.mode = GPIO_MODE_OUTPUT;
				GPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
				GPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
				GPIOConfig.intr_type = GPIO_INTR_DISABLE;
				gpio_config(&GPIOConfig);

				for (auto& unitAndPin : _MPUs)
					gpio_set_level(unitAndPin.pin, true);

				for (auto& unitAndPin : _BMPs)
					gpio_set_level(unitAndPin.pin, true);

				// MPUs
				for (auto& unitAndPin : _MPUs) {
					unitAndPin.unit.setup(
						constants::spi::device,
						unitAndPin.pin,
						10'000'000
					);
				}

				// BMPs
				for (auto& unitAndPin : _BMPs) {
					unitAndPin.unit.setup(
						constants::spi::device,
						unitAndPin.pin,
						10'000'000,

						BMP280Mode::normal,
						BMP280Oversampling::x16,
						BMP280Oversampling::x2,
						BMP280Filter::x4,
						BMP280StandbyDuration::ms1
					);
				}

				xTaskCreate(
					[](void* arg) {
						reinterpret_cast<AHRS*>(arg)->taskBody();
					},
					"ahrs",
					4096,
					this,
					10,
					nullptr
				);
			}

			float getPressure() const {
				return _pressure;
			}

			float getTemperature() const {
				return _temperature;
			}

			float getAltitude() const {
				return _altitude;
			}

		private:
			std::array<AHRSUnitAndSSPin<MPU9250>, 1> _MPUs {
				AHRSUnitAndSSPin<MPU9250> {
					constants::adiru1::mpu9250ss
				}
			};

			std::array<AHRSUnitAndSSPin<BMP280>, 1> _BMPs {
				AHRSUnitAndSSPin<BMP280> {
					constants::adiru1::bmp280ss
				}
			};

			float _pressure = 0;
			float _temperature = 0;
			float _altitude = 0;

			static float computeAltitude(
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

			void updateBMPs() {
				float pressureSum = 0;
				float temperatureSum = 0;

				float pressure;
				float temperature;

				for (auto& BMP : _BMPs) {
					BMP.unit.readPressureAndTemperature(pressure, temperature);

					pressureSum += pressure;
					temperatureSum += temperature;
				}

				_pressure = pressureSum / _BMPs.size();
				_temperature = temperatureSum / _BMPs.size();
				_altitude = computeAltitude(pressureSum, temperatureSum);
			}

			void taskBody() {
				while (true) {
					updateBMPs();

					ESP_LOGI("AHRS", "Avg press: %f, temp: %f, alt: %f", _pressure, _temperature, _altitude);

					vTaskDelay(pdMS_TO_TICKS(1000));
				}
			}
	};
}