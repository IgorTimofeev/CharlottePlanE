#pragma once

#include <array>
#include <cmath>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>

#include "constants.h"
#include "hardware/ADIRS/MPU9250.h"
#include "utils/math.h"

namespace pizda {
	class AdaptiveComplimentaryFiler {
		public:
			static void apply(
				const Vector3F& accelData,
				const Vector3F& gyroData,
				const Vector3F& magData,

				float deltaTimeS,

				float accelGyroTrustFactorMin,
				float accelGyroTrustFactorMax,

				float magGyroTrustFactorMin,
				float magGyroTrustFactorMax,

				float& rollRad,
				float& pitchRad,
				float& yawRad,

				bool log
			) {
				const float accelMagnitude = accelData.getLength();

				// float aRoll = std::atan2(a.getZ(), a.getX());
				// float aPitch = std::atan2(a.getY(), a.getZ());

				float accelRoll = -std::atan2(accelData.getX(), std::sqrt(accelData.getY() * accelData.getY() + accelData.getZ() * accelData.getZ()));
				float accelPitch = std::atan2(accelData.getY(), std::sqrt(accelData.getX() * accelData.getX() + accelData.getZ() * accelData.getZ()));

				float gyroRoll = rollRad + toRadians(gyroData.getY()) * deltaTimeS;
				float gyroPitch = pitchRad + toRadians(gyroData.getX()) * deltaTimeS;
				float gyroYaw = yawRad + toRadians(gyroData.getX()) * deltaTimeS;

				// Filter itself

				// More acceleration -> more gyro trust factor
				float gyroTrustFactor = getGyroTrustFactor(accelGyroTrustFactorMin, accelGyroTrustFactorMax, accelMagnitude);
//					const gTrustFactor = 0;

				rollRad = applyGyroTrustFactor(gyroRoll, accelRoll, gyroTrustFactor);
				pitchRad = applyGyroTrustFactor(gyroPitch, accelPitch, gyroTrustFactor);

				// Mag tilt compensation using computed pitch/roll
				float magYawWithoutTilt = std::atan2(magData.getX(), magData.getY());

				const auto magDataTilt = applyTiltCompensation(magData, rollRad, pitchRad);
				float magYaw = std::atan2(magDataTilt.getX(), magDataTilt.getY());

				// For mag, we're using other gyro trust factor, because mag produces a lot of noise
				gyroTrustFactor = getGyroTrustFactor(magGyroTrustFactorMin, magGyroTrustFactorMax, accelMagnitude);
				yawRad = applyGyroTrustFactor(gyroYaw, magYaw, gyroTrustFactor);

				if (log) {
					ESP_LOGI("Compl", "acc: %f x %f x %f, magni: %f", accelData.getX(), accelData.getY(), accelData.getZ(), accelMagnitude);
					ESP_LOGI("Compl", "gyr: %f x %f x %f", gyroData.getX(), gyroData.getY(), gyroData.getZ());
					ESP_LOGI("Compl", "mag: %f x %f x %f", magData.getX(), magData.getY(), magData.getZ());
					ESP_LOGI("Compl", "mag cor: %f x %f x %f", magDataTilt.getX(), magDataTilt.getY(), magDataTilt.getZ());
					ESP_LOGI("Compl", "mag yaw no tilt: %f", toDegrees(magYawWithoutTilt));
					ESP_LOGI("Compl", "mag yaw: %f", toDegrees(magYaw));
				}
			}

			static Vector3F applyTiltCompensation(const Vector3F& vec, float rollRad, float pitchRad) {
				auto result = vec.rotateAroundXAxis(pitchRad);
				return result.rotateAroundYAxis(rollRad);
			}

		private:
			static float getGyroTrustFactor(float trustFactorMin, float trustFactorMax, float accelMagnitude) {
				// Normally accel magnitude should ~= 1G
				const float accelError = std::abs(accelMagnitude - 1);
				// Let error threshold also be 1G
				const float accelErrorThreshold = 1;
				const float accelMagnitudeFactor = std::clamp(accelError / accelErrorThreshold, 0.0f, 1.0f);

				return trustFactorMin + (trustFactorMax - trustFactorMin) * accelMagnitudeFactor;
			}

			static float applyGyroTrustFactor(float gyroValue, float nonGyroValue, float gyroTrustFactor) {
				return gyroTrustFactor * gyroValue + (1.0f - gyroTrustFactor) * nonGyroValue;
			}
	};

	class IMU {
		public:
			IMU() {

			}

			constexpr static const char* _logTag = "IMU";

			constexpr static uint8_t SRD = 4;

			constexpr static uint16_t FIFOLength = 512;
			constexpr static uint16_t FIFOSampleRateHz = 1000 / (1 + SRD);
			constexpr static float FIFOSampleIntervalS = 1.0f / FIFOSampleRateHz;

			constexpr static MPU9250_fifo_data_source FIFODataSource = MPU9250_FIFO_DATA_SOURCE_ACCEL_GYRO;
			// 3 axis * 2 bytes
			constexpr static uint8_t FIFOSampleDataTypeLength = 3 * 2;
			// Aacc + gyro
			constexpr static uint8_t FIFOSampleDataTypes = 2;
			constexpr static uint8_t FIFOSampleLength = FIFOSampleDataTypeLength * FIFOSampleDataTypes;
			constexpr static uint16_t FIFOMaxSampleCount = FIFOLength / FIFOSampleLength;

			constexpr static uint32_t bytesPerSecond = FIFOSampleLength * FIFOSampleRateHz;
			constexpr static uint32_t minimumReadTaskDelayMs = FIFOLength * 1'000 / bytesPerSecond;
			constexpr static uint32_t safeReadTaskDelayMs = minimumReadTaskDelayMs * 9 / 10;

			MPU9250 MPU {};
			Vector3F accelBias {};
			Vector3F gyroBias {};

			// Mag
			constexpr static uint32_t magSampleRateHz = 100;
			constexpr static uint32_t magSampleIntervalUs = 1'000'000 / magSampleRateHz;

			uint32_t magSampleTimeUs = 0;
			Vector3F magBias {};
			Vector3F lastMagData {};

			float rollRad = 0;
			float pitchRad = 0;
			float yawRad = 0;

			Vector3F accelPosM {};
			Vector3F accelVelocityMs {};

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

//				accelBias = {
//					0.080503,
//					0.092216,
//					-0.210539
//				};
//
//				gyroBias = {
//					-3.064394,
//					-1.297578,
//					-0.379190
//				};
//
//				setMPUOperationalMode();
//
//				return;

				// Using higher attenuation during calibration process
				setMPUCalibrationMode();

				// Accumulating acc/gyr data
				Vector3F aSum {};
				Vector3F gSum {};

				for (uint16_t i = 0; i < iterations; ++i) {
					aSum += MPU.getAccelData();
					gSum += MPU.getGyroData();

					vTaskDelay(pdMS_TO_TICKS(std::max<uint32_t>(1'000 / FIFOSampleRateHz, portTICK_PERIOD_MS)));
				}

				aSum /= static_cast<float>(iterations);
				// Z axis - 1G
				aSum.setZ(aSum.getZ() - 1);

				gSum /= static_cast<float>(iterations);

				ESP_LOGI(_logTag, "acc offset: %f x %f x %f", aSum.getX(), aSum.getY(), aSum.getZ());
				ESP_LOGI(_logTag, "gyr offset: %f x %f x %f", gSum.getX(), gSum.getY(), gSum.getZ());

				accelBias = aSum;
				gyroBias = gSum;

				// Restoring attenuation to operational
				setMPUOperationalMode();
			}

			void calibrateMag() {
				ESP_LOGI(_logTag, "Mag calibration started");

				magBias = {
					(-31 + 74) / 2,
					(-38 + 65) / 2,
					(-76 + 27) / 2
				};
			}

			void tick() {
				// Mag update should be separated from FIFO
				if (esp_timer_get_time() >= magSampleTimeUs) {
					const auto magData = MPU.getMagData();

					ESP_LOGI(_logTag, "mag: %f x %f x %f", magData.getX(), magData.getY(), magData.getZ());

					// Axis swap, fuck MPU
					lastMagData = {
						magData.getY() - magBias.getY(),
						magData.getX() - magBias.getX(),
						-(magData.getZ() - magBias.getZ())
					};

					magSampleTimeUs = esp_timer_get_time() + magSampleIntervalUs;
				}

				const auto sampleCount = MPU.getFIFOCount() / FIFOSampleLength;

				if (sampleCount < 8) {
					ESP_LOGI(_logTag, "FIFO sample count %d is not enough, skipping for more data", sampleCount);
					return;
				}
				else if (sampleCount >= FIFOMaxSampleCount) {
					ESP_LOGW(_logTag, "FIFO sample count %d exceeds max sample count, data was permanently lost", sampleCount);
				}

				MPU.setFIFODataSource(MPU9250_FIFO_DATA_SOURCE_NONE);

//				ESP_LOGI(_logTag, "FIFO sample count: %d", sampleCount);

				const auto samplesToRead = std::min<uint16_t>(FIFOMaxSampleCount, sampleCount);

				uint8_t sample[FIFOSampleLength] {};

				for (uint32_t i = 0; i < samplesToRead; i++) {
					MPU.getFIFOData(sample, FIFOSampleLength);

					const auto accelData = MPU.getAccelData(sample) - accelBias;
					const auto gyroData = MPU.getGyroData(sample + FIFOSampleDataTypeLength) - gyroBias;

					// Applying adaptive complimentary filter
					AdaptiveComplimentaryFiler::apply(
						accelData,
						gyroData,
						lastMagData,

						FIFOSampleIntervalS,

						0.8,
						0.95,

						0.95,
						0.99,

						rollRad,
						pitchRad,
						yawRad,

						i == 0
					);

					// Position
					auto accelTilt = AdaptiveComplimentaryFiler::applyTiltCompensation(accelData, rollRad, pitchRad);
					// Subtracting 1G
					accelTilt.setZ(accelTilt.getZ() - 1);

					constexpr static float GMs2 = 9.80665f;
					auto accelerationMs2 = accelTilt * GMs2;
					auto velocityMs = accelerationMs2 * FIFOSampleIntervalS;
					accelVelocityMs += velocityMs;

					auto accelPositionOffsetM = accelVelocityMs * FIFOSampleIntervalS;
					accelPosM += accelPositionOffsetM;
				}

				ESP_LOGI(_logTag, "Velocity: %f x %f x %f", accelVelocityMs.getX(), accelVelocityMs.getY(), accelVelocityMs.getZ());
				ESP_LOGI(_logTag, "Pos: %f x %f x %f", accelPosM.getX(), accelPosM.getY(), accelPosM.getZ());
				ESP_LOGI(_logTag, "Roll pitch yaw: %f x %f x %f", toDegrees(rollRad), toDegrees(pitchRad), toDegrees(yawRad));

				MPU.resetFIFO();
				MPU.setFIFODataSource(FIFODataSource);
				MPU.readAndClearInterruptStatus();
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

				MPU.setFIFODataSource(FIFODataSource);
			}
	};
}