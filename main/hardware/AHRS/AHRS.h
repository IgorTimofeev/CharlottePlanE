#pragma once

#include <array>
#include <cmath>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "constants.h"
#include "hardware/AHRS/MPU9250.h"
#include "hardware/AHRS/BMP280.h"

#include "logger.h"

namespace pizda {
	template<typename TUnit>
	class AHRSUnit {
		public:
			explicit AHRSUnit(uint8_t address) : address(address) {

			}

			TUnit unit {};
			I2CBusStream stream {};
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

				if (state != ESP_OK && state != ESP_ERR_INVALID_STATE) {
					Logger::check(_logTag, state);
					return;
				}

				// MPUs
				for (auto& MPU : _MPUs) {
					if (!MPU.unit.setup(
						I2CBusHandle,
						MPU.address
					)) {
						Logger::info(_logTag, "MPU-9250 initialization failed");
						return;
					}

					MPU.unit.setAccOffsets(Vector3F(0, 0, 0));
					MPU.unit.setGyrOffsets(Vector3F(0, 0, 0));

					//myMPU9250.setSampleRateDivider(5);

					MPU.unit.setAccRange(MPU9250_ACC_RANGE_2G);
					MPU.unit.setGyrRange(MPU9250_GYRO_RANGE_250);

					MPU.unit.enableAccDLPF(true);
					MPU.unit.setAccDLPF(MPU9250_DLPF_6);

					MPU.unit.enableGyrDLPF();
					MPU.unit.setGyrDLPF(MPU9250_DLPF_6);
				}

				// BMPs
				for (auto& BMP : _BMPs) {
					BMP.stream.setup(I2CBusHandle, BMP.address, 1000000);

					if (!BMP.unit.setup(
						&BMP.stream,

						BMP280Mode::normal,
						BMP280Oversampling::x16,
						BMP280Oversampling::x2,
						BMP280Filter::x4,
						BMP280StandbyDuration::ms125
					)) {
						Logger::info(_logTag, "BMP280 initialization failed");
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
			constexpr static const char* _logTag = "AHRS";
			
			std::array<AHRSUnit<MPU9250>, 1> _MPUs {
				AHRSUnit<MPU9250> {
					constants::adiru1::mpu9250Address
				}
			};

			std::array<AHRSUnit<BMP280>, 1> _BMPs {
				AHRSUnit<BMP280> {
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

				Logger::info(_logTag, "Avg press: %f, temp: %f, alt: %f", _pressure, _temperature, _altitude);
			}

			void updateMPU() {
				for (auto& MPU : _MPUs) {
					Vector3F v1 = MPU.unit.getGValues();
					Vector3F v2 = MPU.unit.readGyroValues();
					Vector3F v3 = MPU.unit.getMagValues();

					Logger::info(_logTag, "MPU acc: %f x %f x %f", v1.getX(), v1.getY(), v1.getZ());
					Logger::info(_logTag, "MPU gyr: %f x %f x %f", v2.getX(), v2.getY(), v2.getZ());
					Logger::info(_logTag, "MPU mag: %f x %f x %f", v3.getX(), v3.getY(), v3.getZ());
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