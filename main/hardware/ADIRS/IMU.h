#pragma once

#include <array>
#include <cmath>
#include <algorithm>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "config.h"
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

				float accelRoll = -std::atan2(accelData.getX(), accelData.getZ());
				float accelPitch = std::atan2(accelData.getY(), accelData.getZ());

//				float accelRoll = -std::atan2(accelData.getX(), std::sqrt(accelData.getY() * accelData.getY() + accelData.getZ() * accelData.getZ()));
//				float accelPitch = std::atan2(accelData.getY(), std::sqrt(accelData.getX() * accelData.getX() + accelData.getZ() * accelData.getZ()));

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
//				float magYawWithoutTilt = std::atan2(magData.getX(), magData.getY());

				const auto magDataTilt = applyTiltCompensation(magData, rollRad, pitchRad);
				float magYaw = std::atan2(magDataTilt.getX(), magDataTilt.getY());

				// For mag, we're using other gyro trust factor, because mag produces a lot of noise
				gyroTrustFactor = getGyroTrustFactor(magGyroTrustFactorMin, magGyroTrustFactorMax, accelMagnitude);
				yawRad = applyGyroTrustFactor(gyroYaw, magYaw, gyroTrustFactor);

				if (log) {
//					ESP_LOGI("Compl", "acc roll/pitch: %f x %f", toDegrees(accelRoll), toDegrees(accelPitch));
//					ESP_LOGI("Compl", "acc: %f x %f x %f, magni: %f", accelData.getX(), accelData.getY(), accelData.getZ(), accelMagnitude);
//					ESP_LOGI("Compl", "gyr: %f x %f x %f", gyroData.getX(), gyroData.getY(), gyroData.getZ());
//					ESP_LOGI("Compl", "mag: %f x %f x %f", magData.getX(), magData.getY(), magData.getZ());
//					ESP_LOGI("Compl", "mag cor: %f x %f x %f", magDataTilt.getX(), magDataTilt.getY(), magDataTilt.getZ());
//					ESP_LOGI("Compl", "mag yaw no tilt: %f", toDegrees(magYawWithoutTilt));
//					ESP_LOGI("Compl", "mag yaw: %f", toDegrees(magYaw));
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

			constexpr static uint32_t FIFOBytesPerSecond = FIFOSampleLength * FIFOSampleRateHz;
			constexpr static uint32_t FIFOMinimumSampleIntervalUs = FIFOLength * 1'000'000 / FIFOBytesPerSecond;
			constexpr static uint32_t FIFOSafeSampleIntervalUs = FIFOMinimumSampleIntervalUs * 9 / 10;
			
			MPU9250 MPU {};
			Vector3F accelBias {};
			Vector3F gyroBias {};

			// Mag
			constexpr static uint32_t magSampleRateHz = 100;
			constexpr static uint32_t magSampleIntervalUs = 1'000'000 / magSampleRateHz;
			constexpr static uint8_t magSampleAveragingCount = 8;

			uint32_t magSampleTimeUs = 0;
			Vector3F magBias {};
			
			uint8_t magSampleCount = 0;
			Vector3F magSampleSum {};
			Vector3F magSampleLast { 0, 1, 0};
			
			// FIFO
			uint32_t FIFOSampleTimeUs = 0;
			
			Vector3F accelPosM {};
			Vector3F accelVelocityMs {};
			
			float rollRad = 0;
			float pitchRad = 0;
			float yawRad = 0;
			
			bool setup(BusStream* busStream) {
				if (!MPU.setup(busStream))
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

				ESP_LOGI(_logTag, "accel and gyro calibration started");

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
				
				accelBias = aSum;
				gyroBias = gSum;
				
				ESP_LOGI(_logTag, "acc bias: %f x %f x %f", accelBias.getX(), accelBias.getY(), accelBias.getZ());
				ESP_LOGI(_logTag, "gyr bias: %f x %f x %f", gyroBias.getX(), gyroBias.getY(), gyroBias.getZ());
				
				// Restoring attenuation to operational
				setMPUOperationalMode();
			}

			void calibrateMag() {
				magBias = {
					11.828777,
					24.903629,
					-25.226059
				};

				return;
				
				ESP_LOGI(_logTag, "mag calibration started");
				
				constexpr static uint32_t durationUs = 10'000'000;
				const auto deadline = esp_timer_get_time() + durationUs;
				
				Vector3F min {};
				Vector3F max {};
				
				do {
					const auto magData = MPU.getMagData();
					
					min.setX(std::min(min.getX(), magData.getX()));
					min.setY(std::min(min.getY(), magData.getY()));
					min.setZ(std::min(min.getZ(), magData.getZ()));
					
					max.setX(std::max(max.getX(), magData.getX()));
					max.setY(std::max(max.getY(), magData.getY()));
					max.setZ(std::max(max.getZ(), magData.getZ()));
					
					vTaskDelay(pdMS_TO_TICKS(std::max(magSampleIntervalUs / 1000, portTICK_PERIOD_MS)));
				}
				while (esp_timer_get_time() < deadline);
				
				magBias.setX(min.getX() + (max.getX() - min.getX()) / 2);
				magBias.setY(min.getY() + (max.getY() - min.getY()) / 2);
				magBias.setZ(min.getZ() + (max.getZ() - min.getZ()) / 2);
				
				ESP_LOGI(_logTag, "mag bias: %f x %f x %f", magBias.getX(), magBias.getY(), magBias.getZ());
			}
			
			void tick() {
				magTick();
				FIFOTick();
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
			
			void magTick() {
				if (esp_timer_get_time() < magSampleTimeUs)
					return;
				
				const auto magData = MPU.getMagData();

//					ESP_LOGI(_logTag, "mag: %f x %f x %f", magData.getX(), magData.getY(), magData.getZ());

				// Axis swap, fuck MPU
				magSampleSum.setX(magSampleSum.getX() + magData.getY() - magBias.getY());
				magSampleSum.setY(magSampleSum.getY() + magData.getX() - magBias.getX());
				magSampleSum.setZ(magSampleSum.getZ() - (magData.getZ() - magBias.getZ()));
				
				magSampleCount++;
				
				// Need to average
				if (magSampleCount >= magSampleAveragingCount) {
					magSampleLast = magSampleSum / magSampleCount;
					magSampleSum = { 0, 0, 0};
					magSampleCount = 0;
				}
				
				magSampleTimeUs = esp_timer_get_time() + magSampleIntervalUs;
			}
			
			void FIFOTick() {
				if (esp_timer_get_time() < FIFOSampleTimeUs)
					return;
				
				const auto sampleCount = MPU.getFIFOCount() / FIFOSampleLength;

//				ESP_LOGI(_logTag, "FIFO sample count %d", sampleCount);
				
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
						magSampleLast,
						
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

//				ESP_LOGI(_logTag, "Velocity: %f x %f x %f", accelVelocityMs.getX(), accelVelocityMs.getY(), accelVelocityMs.getZ());
//				ESP_LOGI(_logTag, "Pos: %f x %f x %f", accelPosM.getX(), accelPosM.getY(), accelPosM.getZ());
//				ESP_LOGI(_logTag, "Roll pitch yaw: %f x %f x %f", toDegrees(rollRad), toDegrees(pitchRad), toDegrees(yawRad));
				
				MPU.resetFIFO();
				MPU.setFIFODataSource(FIFODataSource);
				MPU.readAndClearInterruptStatus();
				
				FIFOSampleTimeUs = esp_timer_get_time() + FIFOSafeSampleIntervalUs;
			}
		
	};
}