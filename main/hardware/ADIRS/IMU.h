#pragma once

#include <array>
#include <cmath>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "constants.h"
#include "hardware/ADIRS/MPU9250.h"

namespace pizda {
	class IMU {
		public:
			IMU() {

			}

			constexpr static const char* _logTag = "IMU";

			constexpr static uint8_t SRD = 4;
			constexpr static uint16_t sampleRateHz = 1000 / (1 + SRD);
			constexpr static float sampleIntervalS = 1.0f / sampleRateHz;

			constexpr static uint16_t FIFOBufferLength = 512;
			// 3 axis * 2 bytes * 2 value types (acc & gyro)
			constexpr static uint8_t FIFOBufferSampleLength = 3 * 2 * 2;
			constexpr static uint16_t FIFOBufferMaxSampleCount = FIFOBufferLength / FIFOBufferSampleLength;

			constexpr static uint32_t bytesPerSecond = FIFOBufferSampleLength * sampleRateHz;
			constexpr static uint32_t minimumReadTaskDelayMs = FIFOBufferLength * 1'000 / bytesPerSecond;
			constexpr static uint32_t safeReadTaskDelayMs = minimumReadTaskDelayMs * 9 / 10;

			MPU9250 MPU {};
			Vector3F aBias {};
			Vector3F gBias {};
			Vector3F mBias {};

			bool setup(i2c_master_bus_handle_t I2CBusHandle, uint8_t I2CAddress) {
				if (!MPU.setup(I2CBusHandle, I2CAddress))
					return false;

				// SRD
				MPU.setSRD(SRD);

//				setMPUOperationalMode();
				calibrateAccAndGyr();
				calibrateMag();

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
			void calibrateAccAndGyr() {
				constexpr static uint16_t iterations = 512;

				ESP_LOGI(_logTag, "Acc and gyr calibration started");

				// Using higher attenuation during calibration process
				setMPUCalibrationMode();

				// Accumulating acc/gyr data
				Vector3F aSum {};
				Vector3F gSum {};

				for (uint16_t i = 0; i < iterations; ++i) {
					aSum += MPU.getAccelData();
					gSum += MPU.getGyroValues();

					vTaskDelay(pdMS_TO_TICKS(std::max<uint32_t>(1'000 / sampleRateHz, portTICK_PERIOD_MS)));
				}

				aSum /= static_cast<float>(iterations);
				// Z axis - 1G
				aSum.setZ(aSum.getZ() - 1);

				gSum /= static_cast<float>(iterations);

				ESP_LOGI(_logTag, "acc offset: %f x %f x %f", aSum.getX(), aSum.getY(), aSum.getZ());
				ESP_LOGI(_logTag, "gyr offset: %f x %f x %f", gSum.getX(), gSum.getY(), gSum.getZ());

				aBias = aSum;
				gBias = gSum;

				// Restoring attenuation to operational
				setMPUOperationalMode();
			}

			void calibrateMag() {
				ESP_LOGI(_logTag, "Mag calibration started");

				mBias = {
					(2 + 30) / 2,
					(15 + 40) / 2,
					(-80 + 28) / 2
				};
			}

			float rollRad = 0;
			float pitchRad = 0;
			float yawRad = 0;

			Vector3F accPos {};
			Vector3F accVelocity {};

			void tick() {
				const auto sampleCount = MPU.getFIFOCount() / FIFOBufferSampleLength;

				if (sampleCount < 4) {
					ESP_LOGI(_logTag, "FIFO sample count is not enough, skipping for more data");
					return;
				}
				else if (sampleCount >= FIFOBufferMaxSampleCount) {
					ESP_LOGW(_logTag, "FIFO sample count exceeds max sample count, data was permanently lost");
				}

				MPU.setFIFODataSource(MPU9250_FIFO_DATA_SOURCE_NONE);
				// Only for cont mode
				// mpu.findFIFOBegin();

				ESP_LOGI(_logTag, "FIFO sample count: %d", sampleCount);

				const auto samplesToRead = std::min<uint16_t>(FIFOBufferMaxSampleCount, sampleCount);

				for (uint32_t i = 0; i < samplesToRead; i++) {
					const auto a = MPU.getAccelDataFromFIFO() - aBias;
					const auto g = MPU.getGyroValuesFromFIFO() - gBias;

					constexpr static float G = 9.80665f;
					auto accelerationMs2 = a * G;
					auto velocityMs = accelerationMs2 * sampleIntervalS;
					accVelocity += velocityMs;

					accPos += accVelocity * sampleIntervalS;

					// Complimentary filter
					{
						// float aRoll = std::atan2(a.getZ(), a.getX());
						// float aPitch = std::atan2(a.getY(), a.getZ());

						// Axis inversion?
						float aRoll = -std::atan2(a.getX(), std::sqrt(a.getY() * a.getY() + a.getZ() * a.getZ()));
						float aPitch = std::atan2(a.getY(), std::sqrt(a.getX() * a.getX() + a.getZ() * a.getZ()));

						float gRoll = rollRad + degToRad(g.getY()) * sampleIntervalS;
						float gPitch = pitchRad + degToRad(g.getX()) * sampleIntervalS;

						// More acceleration -> more gyro trust factor
						constexpr static float gTrustFactorMin = 0.94;
						constexpr static float gTrustFactorMax = 0.99;

						const float aMagnitude = a.getLength();
						// (Acc magnitude - 1G of gravity) / acc range G max
						const float gTrustFactorAMagnitudeFactor = std::clamp((aMagnitude - 1) / 2, 0.0f, 1.0f);
						const float gTrustFactor =
							gTrustFactorMin + (gTrustFactorMax - gTrustFactorMin) * gTrustFactorAMagnitudeFactor;

						rollRad = gTrustFactor * gRoll + (1.0f - gTrustFactor) * aRoll;
						pitchRad = gTrustFactor * gPitch + (1.0f - gTrustFactor) * aPitch;

						if (i == 0) {
//							ESP_LOGI(_logTag, "aMagnitude: %f, gTrustFactorAMagnitudeFactor: %f, gTrustFactor: %f", aMagnitude, gTrustFactorAMagnitudeFactor, gTrustFactor);
							ESP_LOGI(_logTag, "data set %d, acc: %f x %f x %f", i, a.getX(), a.getY(), a.getZ());
							ESP_LOGI(_logTag, "data set %d, gyr: %f x %f x %f", i, g.getX(), g.getY(), g.getZ());
						}
					}
				}

				MPU.resetFIFO();
				MPU.setFIFODataSource(MPU9250_FIFO_DATA_SOURCE_ACCEL_GYRO);
				MPU.readAndClearInterruptStatus();

				// Magnetometer
				auto mag = MPU.readMagData();

				ESP_LOGI(_logTag, "Mag raw: %f x %f x %f", mag.getX(), mag.getY(), mag.getZ());

				// Axis swap, fuck MPU
				auto magCorr = Vector3F(
					mag.getX() - mBias.getX(),
					mag.getY() - mBias.getY(),
					mag.getZ() - mBias.getZ()
				);

				ESP_LOGI(_logTag, "Mag corr: %f x %f x %f", magCorr.getX(), magCorr.getY(), magCorr.getZ());

				auto yawRadSimple = std::atan2(magCorr.getY(), magCorr.getX());

				magCorr = magCorr.rotateAroundYAxis(pitchRad);
				magCorr = magCorr.rotateAroundXAxis(rollRad);

				yawRad = std::atan2(magCorr.getY(), magCorr.getX());

//				ESP_LOGI(_logTag, "Pos: %f x %f x %f", accPos.getX(), accPos.getY(), accPos.getZ());
				ESP_LOGI(_logTag, "Roll pitch yaw: %f x %f x %f, simple: %f", radToDeg(rollRad), radToDeg(pitchRad), radToDeg(yawRad), radToDeg(yawRadSimple));
			}

			float degToRad(float deg) {
				return deg * std::numbers::pi_v<float> / 180.f;
			}

			float radToDeg(float rad) {
				return rad * 180.f / std::numbers::pi_v<float>;
			}

		private:
			void setMPUCalibrationMode() {
				MPU.setGyroRange(MPU9250_GYRO_RANGE_250);
				MPU.setAccelRange(MPU9250_ACC_RANGE_2G);

				MPU.setAccelDLPF(MPU9250_DLPF_6);
				MPU.enableAccelDLPF();

				MPU.setGyroDLPF(MPU9250_DLPF_6);
				MPU.enableGyroDLPF();

				vTaskDelay(pdMS_TO_TICKS(100));

				MPU.disableFIFO();
			}

			void setMPUOperationalMode() {
				// Range
				MPU.setAccelRange(MPU9250_ACC_RANGE_2G);
				MPU.setGyroRange(MPU9250_GYRO_RANGE_250);

				// LPF
				MPU.setAccelDLPF(MPU9250_DLPF_2);
				MPU.enableAccelDLPF();

				MPU.setGyroDLPF(MPU9250_DLPF_2);
				MPU.enableGyroDLPF();

				vTaskDelay(pdMS_TO_TICKS(100));

				// FIFO
				MPU.setFIFOMode(MPU9250_STOP_WHEN_FULL);
				MPU.enableFIFO();

				// In some cases a delay after enabling FIFO makes sense
				vTaskDelay(pdMS_TO_TICKS(100));

				MPU.setFIFODataSource(MPU9250_FIFO_DATA_SOURCE_ACCEL_GYRO);
			}
	};
}