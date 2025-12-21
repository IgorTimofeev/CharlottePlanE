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
			// 3 axis * 2 bytes
			constexpr static uint8_t FIFOBufferSampleDataTypeLength = 3 * 2;
			// Aacc + gyro + mag
			constexpr static uint8_t FIFOBufferSampleDataTypes = 2;
			constexpr static uint8_t FIFOBufferSampleLength = FIFOBufferSampleDataTypeLength * FIFOBufferSampleDataTypes;
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

				aBias = {
					0.083396,
					0.073584,
					-0.209135
				};

				gBias = {
					-3.204822,
					-1.132801,
					-0.406399
				};

				setMPUOperationalMode();

				return;

				// Using higher attenuation during calibration process
				setMPUCalibrationMode();

				// Accumulating acc/gyr data
				Vector3F aSum {};
				Vector3F gSum {};

				for (uint16_t i = 0; i < iterations; ++i) {
					aSum += MPU.getAccelData();
					gSum += MPU.getGyroData();

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
					(-31 + 74) / 2,
					(-38 + 65) / 2,
					(-76 + 27) / 2
				};
			}

			float rollRad = 0;
			float pitchRad = 0;
			float yawRad = 0;

			Vector3F accPos {};
			Vector3F accVelocity {};

			void tick() {
				const auto sampleCount = MPU.getFIFOCount() / FIFOBufferSampleLength;

				if (sampleCount < 8) {
					ESP_LOGI(_logTag, "FIFO sample count is not enough, skipping for more data");
					return;
				}
				else if (sampleCount >= FIFOBufferMaxSampleCount) {
					ESP_LOGW(_logTag, "FIFO sample count exceeds max sample count, data was permanently lost");
				}

				MPU.setFIFODataSource(MPU9250_FIFO_DATA_SOURCE_NONE);

				ESP_LOGI(_logTag, "FIFO sample count: %d", sampleCount);

				const auto samplesToRead = std::min<uint16_t>(FIFOBufferMaxSampleCount, sampleCount);

				const auto mRaw = MPU.getMagData();

				// Axis swap, fuck MPU
				auto m = Vector3F(
					mRaw.getY() - mBias.getY(),
					mRaw.getX() - mBias.getX(),
					-(mRaw.getZ() - mBias.getZ())
				);

				for (uint32_t i = 0; i < samplesToRead; i++) {
					uint8_t FIFOSampleBuffer[FIFOBufferSampleLength] {};
					MPU.getFIFOData(FIFOSampleBuffer, FIFOBufferSampleLength);

					const auto a = MPU.getAccelData(FIFOSampleBuffer) - aBias;
					const auto g = MPU.getGyroData(FIFOSampleBuffer + FIFOBufferSampleDataTypeLength) - gBias;
//					const auto mRaw = MPU.getMagData(FIFOSampleBuffer + FIFOBufferSampleDataTypeLength);

//					// Axis swap, fuck MPU
//					auto m = Vector3F(
//						mRaw.getY() - mBias.getY(),
//						mRaw.getX() - mBias.getX(),
//						-(mRaw.getZ() - mBias.getZ())
//					);

					constexpr static float G = 9.80665f;
					auto accelerationMs2 = a * G;
					auto velocityMs = accelerationMs2 * sampleIntervalS;
					accVelocity += velocityMs;

					accPos += accVelocity * sampleIntervalS;

					// Complimentary filter
					// float aRoll = std::atan2(a.getZ(), a.getX());
					// float aPitch = std::atan2(a.getY(), a.getZ());

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
					float gTrustFactor = gTrustFactorMin + (gTrustFactorMax - gTrustFactorMin) * gTrustFactorAMagnitudeFactor;

					gTrustFactor = 0;

					rollRad = gTrustFactor * gRoll + (1.0f - gTrustFactor) * aRoll;
					pitchRad = gTrustFactor * gPitch + (1.0f - gTrustFactor) * aPitch;

					// Magnetometer
					auto yawRadSimple = std::atan2(m.getX(), m.getY());

					m = m.rotateAroundXAxis(pitchRad);
					m = m.rotateAroundYAxis(rollRad);

					yawRad = std::atan2(m.getX(), m.getY());

					if (i == 0) {
//				ESP_LOGI(_logTag, "Pos: %f x %f x %f", accPos.getX(), accPos.getY(), accPos.getZ());

						ESP_LOGI(_logTag, "Mag raw: %f x %f x %f", mRaw.getX(), mRaw.getY(), mRaw.getZ());
						ESP_LOGI(_logTag, "Mag cor: %f x %f x %f", m.getX(), m.getY(), m.getZ());
						ESP_LOGI(_logTag, "Yaw simple: %f", radToDeg(yawRadSimple));

//							ESP_LOGI(_logTag, "aMagnitude: %f, gTrustFactorAMagnitudeFactor: %f, gTrustFactor: %f", aMagnitude, gTrustFactorAMagnitudeFactor, gTrustFactor);
						ESP_LOGI(_logTag, "data set %d, acc: %f x %f x %f", i, a.getX(), a.getY(), a.getZ());
						ESP_LOGI(_logTag, "data set %d, gyr: %f x %f x %f", i, g.getX(), g.getY(), g.getZ());
						ESP_LOGI(_logTag, "Roll pitch yaw: %f x %f x %f", radToDeg(rollRad), radToDeg(pitchRad), radToDeg(yawRad));
					}
				}

				MPU.resetFIFO();
				MPU.setFIFODataSource(MPU9250_FIFO_DATA_SOURCE_ACCEL_GYRO);
				MPU.readAndClearInterruptStatus();
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