#pragma once

#include <array>
#include <cmath>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "constants.h"
#include "hardware/AHRS/MPU9250.h"

namespace pizda {
	class IMU {
		public:
			IMU() {

			}

			constexpr static const char* _logTag = "IMU";

			constexpr static uint8_t MPUSRD = 4;
			constexpr static uint16_t MPUFrequencyHz = 1000 / (1 + MPUSRD);
			constexpr static float MPUTickDurationS = 1.0f / MPUFrequencyHz;

			constexpr static uint16_t MPUFIFOBufferLength = 512;
			constexpr static uint8_t MPUFIFOBufferBatchLength = 3 * 2 * 2;
			constexpr static uint16_t MPUFIFOBufferMaxBatchesCount = MPUFIFOBufferLength / MPUFIFOBufferBatchLength;
			constexpr static uint16_t MPUFIFOBufferDangerousBatchesCount = MPUFIFOBufferMaxBatchesCount * 8 / 10;

			MPU9250 MPU {};
			Vector3F accOffset {};
			Vector3F gyrOffset {};

			bool setup(i2c_master_bus_handle_t I2CBusHandle, uint8_t I2CAddress) {
				if (!MPU.setup(I2CBusHandle, I2CAddress))
					return false;

				// SRD
				MPU.setSRD(MPUSRD);

				setMPUOperationalMode();

				// Calibration
//				calibrate();

				return true;
			}

			/* The slope of the curve of acceleration vs measured values fits quite well to the theoretical
			* values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the
			* MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset
			* values. It assumes your MPU9250 is positioned flat with its x,y-plane. The more you deviate
			* from this, the less accurate will be your results.
			* The function also measures the offset of the gyroscope data. The gyroscope offset does not
			* depend on the positioning.
			* This function needs to be called at the beginning since it can overwrite your settings!
			*
			*  There's a more accurate method for calibration. You have to determine the minimum and maximum
			*  raw acceleration values of the axes determined in the range +/- 2 g.
			*  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
			*  Use either autoOffset or setAccOffsets, not both.
			*/
			void calibrate() {
				ESP_LOGI(_logTag, "IMU calibration started");

				constexpr static uint16_t iterations = 512;

				// Using higher attenuation during calibration process
				MPU.setGyrRange(MPU9250_GYRO_RANGE_250);
				MPU.setAccRange(MPU9250_ACC_RANGE_2G);

				MPU.setAccDLPF(MPU9250_DLPF_6);
				MPU.enableAccDLPF();

				MPU.setGyrDLPF(MPU9250_DLPF_6);
				MPU.enableGyrDLPF();

				vTaskDelay(pdMS_TO_TICKS(100));
				
				MPU.disableFIFO();

				// Accumulating acc/gyr data
				Vector3F aAcc {};
				Vector3F gAcc {};

				for (uint16_t i = 0; i < iterations; ++i) {
					aAcc += MPU.readAccValues();
					gAcc += MPU.readGyroValues();

					vTaskDelay(pdMS_TO_TICKS(portTICK_PERIOD_MS));
				}

				aAcc /= iterations;
				gAcc /= iterations;

				ESP_LOGI(_logTag, "acc offset: %f x %f x %f", aAcc.getX(), aAcc.getY(), aAcc.getZ());
				ESP_LOGI(_logTag, "gyr offset: %f x %f x %f", gAcc.getX(), gAcc.getY(), gAcc.getZ());

				accOffset = aAcc;
				gyrOffset = gAcc;

				// Restoring attenuation to operational
				setMPUOperationalMode();
			}

			float rollRad = 0;
			float pitchRad = 0;
			float yawRad = 0;

			Vector3F accPos {};
			Vector3F accVelocity {};

			void tick() {
				const auto dataSetsCount = MPU.readFIFODataSetsCount();

				if (dataSetsCount < 4) {
					ESP_LOGI(_logTag, "FIFO data sets count is not enough, skipping for more data");
					return;
				}
				else if (dataSetsCount >= MPUFIFOBufferDangerousBatchesCount) {
					ESP_LOGI(_logTag, "FIFO data sets count is dangerously big");
				}

				MPU.stopFIFO();
				// Only for cont mode
				// mpu.findFIFOBegin();

				ESP_LOGI(_logTag, "FIFO data sets count: %d", dataSetsCount);

				const auto batchesToRead = std::min<uint16_t>(MPUFIFOBufferMaxBatchesCount, dataSetsCount);

				float gyrTrustFactor = 0.98;

				for (uint32_t i = 0; i < batchesToRead; i++) {
					const auto a = MPU.readAccValuesFromFIFO() - accOffset;
					const auto g = MPU.readGyroValuesFromFIFO() - gyrOffset;

					constexpr static float G = 9.80665f;
					auto accelerationMs2 = a * G;
					auto velocityMs = accelerationMs2 * MPUTickDurationS;
					accVelocity += velocityMs;

					accPos += accVelocity * MPUTickDurationS;

//					ESP_LOGI(_logTag, "data set %d, acc: %f x %f x %f", i, a.getX(), a.getY(), a.getZ());
//					ESP_LOGI(_logTag, "data set %d, gyr: %f x %f x %f", i, g.getX(), g.getY(), g.getZ());

					// Complimentary filter
					float aRoll = std::atan2(a.getY(), std::sqrt(a.getX() * a.getX() + a.getZ() * a.getZ()));
					float aPitch = std::atan2(-a.getX(), std::sqrt(a.getY() * a.getY() + a.getZ() * a.getZ()));

					float gRoll = rollRad + degToRad(g.getX()) * MPUTickDurationS;
					float gPitch = pitchRad + degToRad(g.getY()) * MPUTickDurationS;

					rollRad = gyrTrustFactor * gRoll + (1.0f - gyrTrustFactor) * aRoll;
					pitchRad = gyrTrustFactor * gPitch + (1.0f - gyrTrustFactor) * aPitch;

//					auto v1 = MPU.readAccValues() - accOffset;
//					auto v2 = MPU.readGyroValues() - gyrOffset;
//
//					ESP_LOGI(_logTag, "simple acc: %f x %f x %f", i, v1.getX(), v1.getY(), v1.getZ());
//					ESP_LOGI(_logTag, "simple gyr: %f x %f x %f", i, v2.getX(), v2.getY(), v2.getZ());
				}

				MPU.resetFIFO();
				MPU.startFIFO(MPU9250_FIFO_ACC_GYR);
				MPU.readAndClearInterruptStatus();

				Vector3F v3 = MPU.readMagValues();

				ESP_LOGI(_logTag, "Mag: %f x %f x %f", v3.getX(), v3.getY(), v3.getZ());
				ESP_LOGI(_logTag, "Roll pitch yaw: %f x %f x %f", radToDeg(rollRad), radToDeg(pitchRad), radToDeg(yawRad));
				ESP_LOGI(_logTag, "POs: %f x %f x %f", accPos.getX(), accPos.getY(), accPos.getZ());
			}

			float degToRad(float deg) {
				return deg * std::numbers::pi_v<float> / 180.f;
			}

			float radToDeg(float rad) {
				return rad * 180.f / std::numbers::pi_v<float>;
			}

		private:
			void setMPUOperationalMode() {
				// Range
				MPU.setAccRange(MPU9250_ACC_RANGE_2G);
				MPU.setGyrRange(MPU9250_GYRO_RANGE_250);

				// LPF
				MPU.setAccDLPF(MPU9250_DLPF_2);
				MPU.enableAccDLPF();

				MPU.setGyrDLPF(MPU9250_DLPF_2);
				MPU.enableGyrDLPF();

				vTaskDelay(pdMS_TO_TICKS(100));

				// FIFO
				MPU.setFIFOMode(MPU9250_STOP_WHEN_FULL);
				MPU.enableFIFO();

				// In some cases a delay after enabling FIFO makes sense
				vTaskDelay(pdMS_TO_TICKS(100));

				MPU.startFIFO(MPU9250_FIFO_ACC_GYR);
			}
	};
}