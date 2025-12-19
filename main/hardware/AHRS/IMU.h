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
			constexpr static uint32_t MPUTickDurationUs = 1'000'000 / MPUFrequencyHz;

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

//				setMPUOperationalMode();

				// Calibration
				calibrate();

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
				ESP_LOGI(_logTag, "MPU calibration started");

				constexpr static uint16_t iterations = 512;

				// Highest resolution
				MPU.setGyrRange(MPU9250_GYRO_RANGE_250);
				MPU.setAccRange(MPU9250_ACC_RANGE_2G);

				// Lowest noise
				MPU.setAccDLPF(MPU9250_DLPF_6);
				MPU.enableAccDLPF();

				MPU.setGyrDLPF(MPU9250_DLPF_6);
				MPU.enableGyrDLPF();

				vTaskDelay(pdMS_TO_TICKS(100));

				// FIFO
				MPU.disableFIFO();

				Vector3F aAcc {};
				Vector3F gAcc {};

				for (uint16_t i = 0; i < iterations; ++i) {
					aAcc += MPU.readAccValues();
					gAcc += MPU.readGyroValues();

					vTaskDelay(pdMS_TO_TICKS(portTICK_PERIOD_MS));
				}

				aAcc /= iterations;
				gAcc /= iterations;

				ESP_LOGI(_logTag, "MPU acc offset: %f x %f x %f", aAcc.getX(), aAcc.getY(), aAcc.getZ());
				ESP_LOGI(_logTag, "MPU gyr offset: %f x %f x %f", gAcc.getX(), gAcc.getY(), gAcc.getZ());

				accOffset = aAcc;
				gyrOffset = gAcc;

				setMPUOperationalMode();
			}

			void tick() {
				const auto dataSetsCount = MPU.readFIFODataSetsCount();

				if (dataSetsCount < 4) {
					ESP_LOGI(_logTag, "MPU FIFO data sets count is not enough, skipping for more data");
					return;
				}
				else if (dataSetsCount >= MPUFIFOBufferDangerousBatchesCount) {
					ESP_LOGI(_logTag, "MPU FIFO data sets count is dangerously big");
				}

				MPU.stopFIFO();
				// Only for cont mode
				// mpu.findFIFOBegin();

				ESP_LOGI(_logTag, "MPU FIFO data sets count: %d", dataSetsCount);

				Vector3F a[MPUFIFOBufferMaxBatchesCount] {};
				Vector3F g[MPUFIFOBufferMaxBatchesCount] {};

				const auto batchesToRead = std::min<uint16_t>(MPUFIFOBufferMaxBatchesCount, dataSetsCount);

				for (uint32_t i = 0; i < batchesToRead; i++) {
					a[i] = MPU.readAccValuesFromFIFO() - accOffset;
					g[i] = MPU.readGyroValuesFromFIFO() - gyrOffset;

					ESP_LOGI(_logTag, "data set %d, acc: %f x %f x %f", i, a[i].getX(), a[i].getY(), a[i].getZ());
					ESP_LOGI(_logTag, "data set %d, gyr: %f x %f x %f", i, g[i].getX(), g[i].getY(), g[i].getZ());

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

				ESP_LOGI(_logTag, "MPU mag: %f x %f x %f", v3.getX(), v3.getY(), v3.getZ());
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