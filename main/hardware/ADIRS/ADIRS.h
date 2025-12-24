#pragma once

#include <array>
#include <cmath>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "config.h"
#include "hardware/ADIRS/IMU.h"
#include "hardware/ADIRS/BMP280.h"

namespace pizda {
	template<typename TUnit>
	class ADIRSUnit {
		public:
			explicit ADIRSUnit(uint8_t address) : address(address) {

			}

			TUnit unit {};
			I2CBusStream stream {};
			uint8_t address;
	};

	class ADIRS {
		public:
			void setup() {
				if (!setupBus())
					return;

				if (!setupIMUs())
					return;

				if (!setupBMPs())
					return;

				xTaskCreate(
					[](void* arg) {
						reinterpret_cast<ADIRS*>(arg)->taskBody();
					},
					"ahrs",
					4096,
					this,
					10,
					nullptr
				);
			}
			
			float getRollRad() const {
				return _rollRad;
			}
			
			float getPitchRad() const {
				return _pitchRad;
			}
			
			float getYawRad() const {
				return _yawRad;
			}
			
			
			float getPressurePa() const {
				return _pressurePa;
			}
			
			float getTemperatureC() const {
				return _temperatureC;
			}
			
			float getAltitudeM() const {
				return _altitudeM;
			}
			
		private:
			constexpr static const char* _logTag = "AHRS";
			
			float _rollRad = 0;
			float _pitchRad = 0;
			float _yawRad = 0;
			
			float _pressurePa = 0;
			float _temperatureC = 0;
			float _altitudeM = 0;

			i2c_master_bus_handle_t _I2CBusHandle {};

			std::array<ADIRSUnit<IMU>, 1> _IMUs {
				ADIRSUnit<IMU> {
					config::adiru1::mpu9250Address
				}
			};

			std::array<ADIRSUnit<BMP280>, 1> _BMPs {
				ADIRSUnit<BMP280> {
					config::adiru1::bmp280Address
				}
			};

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

			bool setupBus() {
				i2c_master_bus_config_t i2c_mst_config {};
				i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
				i2c_mst_config.i2c_port = I2C_NUM_0;
				i2c_mst_config.scl_io_num = config::i2c::SCL;
				i2c_mst_config.sda_io_num = config::i2c::SDA;
				i2c_mst_config.glitch_ignore_cnt = 7;
				i2c_mst_config.flags.enable_internal_pullup = true;

				const auto state = i2c_new_master_bus(&i2c_mst_config, &_I2CBusHandle);

				if (state != ESP_OK && state != ESP_ERR_INVALID_STATE) {
					ESP_ERROR_CHECK(state);
					return false;
				}

				return true;
			}

			bool setupIMUs() {
				for (uint8_t i = 0; i < static_cast<uint8_t>(_IMUs.size()); ++i) {
					auto& IMU = _IMUs[i];

					IMU.stream.setup(_I2CBusHandle, IMU.address, 400'000);

					if (!IMU.unit.setup(&IMU.stream)) {
						ESP_LOGE(_logTag, "IMU %d initialization failed", i);
						return false;
					}
				}

				return true;
			}

			void updateIMUs() {
				_rollRad = 0;
				_pitchRad = 0;
				_yawRad = 0;
				
				for (auto& IMU : _IMUs) {
					IMU.unit.tick();
					
					_rollRad += IMU.unit.rollRad;
					_pitchRad += IMU.unit.pitchRad;
					_yawRad += IMU.unit.yawRad;
				}
				
				_rollRad /= _IMUs.size();
				_pitchRad /= _IMUs.size();
				_yawRad /= _IMUs.size();
			}

			bool setupBMPs() {
				for (uint8_t i = 0; i < static_cast<uint8_t>(_BMPs.size()); ++i) {
					auto& BMP = _BMPs[i];

					BMP.stream.setup(_I2CBusHandle, BMP.address, 1000000);

					if (!BMP.unit.setup(
						&BMP.stream,

						BMP280Mode::normal,
						BMP280Oversampling::x16,
						BMP280Oversampling::x2,
						BMP280Filter::x4,
						BMP280StandbyDuration::ms125
					)) {
						ESP_LOGI(_logTag, "BMP %d initialization failed", i);
						return false;
					}
				}

				return true;
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
				
				_pressurePa = pressureSum / _BMPs.size();
				_temperatureC = temperatureSum / _BMPs.size();
				
				_altitudeM = computeAltitude(_pressurePa, _temperatureC);

//				ESP_LOGI(_logTag, "Avg press: %f, temp: %f, alt: %f", _pressure, _temperature, _altitude);
			}

			void taskBody() {
				while (true) {
					updateIMUs();
					updateBMPs();

					vTaskDelay(pdMS_TO_TICKS(IMU::safeReadTaskDelayMs));
//					vTaskDelay(pdMS_TO_TICKS(1000));
				}
			}
	};
}