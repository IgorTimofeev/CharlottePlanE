#pragma once

#include <array>
#include <cmath>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "constants.h"
#include "hardware/AHRS/MPU9250.h"
#include "hardware/AHRS/BMP280.h"

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
					if (!unitAndAddress.unit.init(
						I2CBusHandle,
						unitAndAddress.address
					)) {
						ESP_LOGE("AHRS", "MPU initialization failed");
						return;
					}

					/* The slope of the curve of acceleration vs measured values fits quite well to the theoretical 
   * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the 
   * MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset 
   * values. It assumes your MPU9250 is positioned flat with its x,y-plane. The more you deviate 
   * from this, the less accurate will be your results.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called at the beginning since it can overwrite your settings!
   */
					ESP_LOGI("AHRS", "Position you MPU9250 flat and don't move it - calibrating...");
					unitAndAddress.unit.autoOffsets();
					ESP_LOGI("AHRS", "Done!");

					/*  This is a more accurate method for calibration. You have to determine the minimum and maximum 
					 *  raw acceleration values of the axes determined in the range +/- 2 g. 
					 *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
					 *  Use either autoOffset or setAccOffsets, not both.
					 */
					//myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

					/*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
					 *  Sample rate = Internal sample rate / (1 + divider) 
					 *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
					 *  Divider is a number 0...255
					 */
					//myMPU9250.setSampleRateDivider(5);

					/*  MPU9250_ACC_RANGE_2G      2 g   
					 *  MPU9250_ACC_RANGE_4G      4 g
					 *  MPU9250_ACC_RANGE_8G      8 g   
					 *  MPU9250_ACC_RANGE_16G    16 g
					 */
					unitAndAddress.unit.setAccRange(MPU9250_ACC_RANGE_2G);

					/*  Enable/disable the digital low pass filter for the accelerometer 
					 *  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
					 */
					unitAndAddress.unit.enableAccDLPF(true);

					/*  Digital low pass filter (DLPF) for the accelerometer (if DLPF enabled) 
					  *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7 
					  *   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
					  *     0           460               1.94           1
					  *     1           184               5.80           1
					  *     2            92               7.80           1
					  *     3            41              11.80           1
					  *     4            20              19.80           1
					  *     5            10              35.70           1
					  *     6             5              66.96           1
					  *     7           460               1.94           1
					  */
					unitAndAddress.unit.setAccDLPF(MPU9250_DLPF_6);
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
					Vector3F v1 = MPU.unit.getGValues();
					Vector3F v2 = MPU.unit.getGyrValues();
					Vector3F v3 = MPU.unit.getMagValues();

					ESP_LOGI("AHRS", "MPU acc: %f x %f x %f", v1.getX(), v1.getY(), v1.getZ());
					ESP_LOGI("AHRS", "MPU gyr: %f x %f x %f", v2.getX(), v2.getY(), v2.getZ());
					ESP_LOGI("AHRS", "MPU mag: %f x %f x %f", v3.getX(), v3.getY(), v3.getZ());

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