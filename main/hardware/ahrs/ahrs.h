#pragma once

#include <array>
#include <cmath>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "constants.h"
#include "hardware/ahrs/mpu9250.h"
#include "hardware/ahrs/bmp280.h"

namespace pizda {
	template<typename T>
	class AHRSUnitAndI2CAddress {
		public:
			explicit AHRSUnitAndI2CAddress(uint8_t address) : address(address) {

			}

			T unit {};
			uint8_t address;
	};

	class AHRS {
		public:
			void setup() {
				// I2C
				i2c_master_bus_config_t i2c_mst_config {};
				i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
				i2c_mst_config.i2c_port = I2C_NUM_0;
				i2c_mst_config.scl_io_num = constants::i2c::scl;
				i2c_mst_config.sda_io_num = constants::i2c::sda;
				i2c_mst_config.glitch_ignore_cnt = 7;
				i2c_mst_config.flags.enable_internal_pullup = true;

				i2c_master_bus_handle_t I2CBusHandle;
				const auto state = i2c_new_master_bus(&i2c_mst_config, &I2CBusHandle);
				assert(state == ESP_OK || state == ESP_ERR_INVALID_STATE);

				// MPUs
				for (auto& unitAndAddress : _MPUs) {
					if (!unitAndAddress.unit.setup(
						I2CBusHandle,
						unitAndAddress.address
					)) {
						ESP_LOGE("AHRS", "BMP280 initialization failed");
						return;
					}
				}

				// BMPs
				for (auto& unitAndAddress : _BMPs) {
					if (!unitAndAddress.unit.setup(
						I2CBusHandle,
						unitAndAddress.address,

						BMP280Mode::normal,
						BMP280Oversampling::x16,
						BMP280Oversampling::x2,
						BMP280Filter::x4,
						BMP280StandbyDuration::ms125
					)) {
						ESP_LOGE("AHRS", "BMP280 initialization failed");
						return;
					}
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
			std::array<AHRSUnitAndI2CAddress<MPU9250>, 1> _MPUs {
				AHRSUnitAndI2CAddress<MPU9250> {
					constants::adiru1::mpu9250Address
				}
			};

			std::array<AHRSUnitAndI2CAddress<BMP280>, 1> _BMPs {
				AHRSUnitAndI2CAddress<BMP280> {
					constants::adiru1::bmp280Address
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

				ESP_LOGI("AHRS", "Avg press: %f, temp: %f, alt: %f", _pressure, _temperature, _altitude);
			}

			void updateMPU() {
				for (auto& MPU : _MPUs) {
					if (MPU.unit.Read()) {
						ESP_LOGI("AHRS", "Mpu new_imu_data: %d", MPU.unit.new_imu_data());
						ESP_LOGI("AHRS", "Mpu new_mag_data: %d", MPU.unit.new_mag_data());
						ESP_LOGI("AHRS", "Mpu accel_x_mps2: %f", MPU.unit.accel_x_mps2());
						ESP_LOGI("AHRS", "Mpu accel_y_mps2: %f", MPU.unit.accel_y_mps2());
						ESP_LOGI("AHRS", "Mpu accel_z_mps2: %f", MPU.unit.accel_z_mps2());
						ESP_LOGI("AHRS", "Mpu gyro_x_radps: %f", MPU.unit.gyro_x_radps());
						ESP_LOGI("AHRS", "Mpu gyro_y_radps: %f", MPU.unit.gyro_y_radps());
						ESP_LOGI("AHRS", "Mpu gyro_z_radps: %f", MPU.unit.gyro_z_radps());
						ESP_LOGI("AHRS", "Mpu mag_x_ut: %f", MPU.unit.mag_x_ut());
						ESP_LOGI("AHRS", "Mpu mag_y_ut: %f", MPU.unit.mag_y_ut());
						ESP_LOGI("AHRS", "Mpu mag_z_ut: %f", MPU.unit.mag_z_ut());
						ESP_LOGI("AHRS", "Mpu die_temp_c: %f", MPU.unit.die_temp_c());
					}
					else {
						ESP_LOGI("AHRS", "Mpu unable to read");
					}
				}
			}

			void taskBody() {
				while (true) {
					updateMPU();
					updateBMPs();

					vTaskDelay(pdMS_TO_TICKS(1000));
				}
			}
	};
}