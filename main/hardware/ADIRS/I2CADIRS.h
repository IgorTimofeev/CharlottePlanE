#pragma once

#include "hardware/ADIRS/ADIRS.h"

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
	class I2CADIRSUnit {
		public:
			explicit I2CADIRSUnit(const uint8_t address) : address(address) {

			}

			TUnit unit {};
			I2CBusStream stream {};
			uint8_t address;
	};

	class I2CADIRS : public ADIRS {
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
						static_cast<I2CADIRS*>(arg)->taskBody();
					},
					"ADIRS",
					4096,
					this,
					10,
					nullptr
				);
			}
			
		private:
			i2c_master_bus_handle_t _I2CBusHandle {};

			std::array<I2CADIRSUnit<IMU>, 1> _IMUs {
				I2CADIRSUnit<IMU> {
					config::adiru1::mpu9250Address
				}
			};

			std::array<I2CADIRSUnit<BMP280>, 1> _BMPs {
				I2CADIRSUnit<BMP280> {
					config::adiru1::bmp280Address
				}
			};

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
				float rollRadSum = 0;
				float pitchRadSum = 0;
				float yawRadSum = 0;
				Vector3F accelerationGSum {};
				float accelVelocityMsSum = 0;
				
				for (auto& IMU : _IMUs) {
					IMU.unit.tick();
					
					rollRadSum += IMU.unit.getRollRad();
					pitchRadSum += IMU.unit.getPitchRad();
					yawRadSum += IMU.unit.getYawRad();
					accelerationGSum += IMU.unit.getAccelerationG();
					accelVelocityMsSum += std::abs(IMU.unit.getVelocityMs().getY());
				}
				
				setRollRad(rollRadSum / _IMUs.size());
				setPitchRad(pitchRadSum / _IMUs.size());
				setYawRad(yawRadSum / _IMUs.size());
				updateHeadingFromYaw();
				
				setAccelSpeedMPS(accelVelocityMsSum / _IMUs.size());
				
				const auto accelerationG = accelerationGSum / _IMUs.size();
				updateSlipAndSkidFactor(accelerationG.getX(), IMU::accelerationGMax);
			}

			bool setupBMPs() {
				for (uint8_t i = 0; i < static_cast<uint8_t>(_BMPs.size()); ++i) {
					auto& BMP = _BMPs[i];

					BMP.stream.setup(_I2CBusHandle, BMP.address, 1'000'000);

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
				
				setPressurePa(pressureSum / _BMPs.size());
				setTemperatureC(temperatureSum / _BMPs.size());
				updateAltitudeFromPressureTemperatureAndReferenceValue();
				
//				ESP_LOGI(_logTag, "Avg press: %f, temp: %f, alt: %f", _pressure, _temperature, _altitude);
			}
			
			[[noreturn]] void taskBody() {
				while (true) {
					updateIMUs();
					updateBMPs();

					vTaskDelay(pdMS_TO_TICKS(std::max(IMU::recommendedTickDelayUs / 1000, portTICK_PERIOD_MS)));
//					vTaskDelay(pdMS_TO_TICKS(1000));
				}
			}
	};
}