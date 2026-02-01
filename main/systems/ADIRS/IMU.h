#pragma once

#include <array>
#include <cmath>
#include <algorithm>

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <MPU9250.h>
#include <lowPassFilter.h>

#include "config.h"
#include "utilities/math.h"

namespace pizda {
	class AdaptiveComplimentaryFiler {
		public:
			static void apply(
				const Vector3F& accelData,
				const Vector3F& gyroData,
				const Vector3F& magData,

				const float deltaTimeS,

				const float accelGyroTrustFactorMin,
				const float accelGyroTrustFactorMax,

				const float magGyroTrustFactorMin,
				const float magGyroTrustFactorMax,

				float& rollRad,
				float& pitchRad,
				float& yawRad,

				const bool log
			) {
				const float accelMagnitude = accelData.getLength();

				const float accelRoll = -std::atan2(accelData.getX(), accelData.getZ());
				const float accelPitch = std::atan2(accelData.getY(), accelData.getZ());

//				float accelRoll = -std::atan2(accelData.getX(), std::sqrt(accelData.getY() * accelData.getY() + accelData.getZ() * accelData.getZ()));
//				float accelPitch = std::atan2(accelData.getY(), std::sqrt(accelData.getX() * accelData.getX() + accelData.getZ() * accelData.getZ()));

				const float gyroRoll = rollRad + toRadians(gyroData.getY()) * deltaTimeS;
				const float gyroPitch = pitchRad + toRadians(gyroData.getX()) * deltaTimeS;
				const float gyroYaw = yawRad + toRadians(gyroData.getX()) * deltaTimeS;

				// Filter itself

				// More acceleration -> more gyro trust factor
				float gyroTrustFactor = getGyroTrustFactor(accelGyroTrustFactorMin, accelGyroTrustFactorMax, accelMagnitude);
//					const gTrustFactor = 0;

				rollRad = applyGyroTrustFactor(gyroRoll, accelRoll, gyroTrustFactor);
				pitchRad = applyGyroTrustFactor(gyroPitch, accelPitch, gyroTrustFactor);

				// Mag tilt compensation using computed pitch/roll
//				float magYawWithoutTilt = std::atan2(magData.getX(), magData.getY());

				const auto magDataTilt = applyTiltCompensation(magData, rollRad, pitchRad);
				const auto magYaw = std::atan2(magDataTilt.getX(), magDataTilt.getY());

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

			static Vector3F applyTiltCompensation(const Vector3F& vec, const float rollRad, const float pitchRad) {
				const auto result = vec.rotateAroundXAxis(pitchRad);
				return result.rotateAroundYAxis(rollRad);
			}

		private:
			static float getGyroTrustFactor(const float trustFactorMin, const float trustFactorMax, const float accelMagnitude) {
				// Normally accel magnitude should ~= 1G
				const float accelError = std::abs(accelMagnitude - 1);
				// Let error threshold also be 1G
				constexpr static float accelErrorThreshold = 1;
				const float accelMagnitudeFactor = std::clamp(accelError / accelErrorThreshold, 0.0f, 1.0f);

				return trustFactorMin + (trustFactorMax - trustFactorMin) * accelMagnitudeFactor;
			}

			static float applyGyroTrustFactor(const float gyroValue, const float nonGyroValue, const float gyroTrustFactor) {
				return gyroTrustFactor * gyroValue + (1.0f - gyroTrustFactor) * nonGyroValue;
			}
	};

	class IMU {
		public:
			bool setup(busHAL* bus) {
				if (!_MPU.setup(bus))
					return false;

				// SRD
				_MPU.setSRD(MPUSRD);

				setOperationalMode();

				return true;
			}

			constexpr static uint8_t MPUSRD = 4;
			constexpr static uint16_t MPUSampleRateHz = 1000 / (1 + MPUSRD);
			constexpr static uint32_t MPUSampleIntervalHz = 1'000'000 / MPUSampleRateHz;

			constexpr static uint32_t magSampleRateHz = 100;
			constexpr static uint32_t magSampleIntervalUs = 1'000'000 / magSampleRateHz;

			void tick() {
				magTick();
				FIFOTick();
			}
			
			float getRollRad() const {
				return rollRad;
			}
			
			float getPitchRad() const {
				return pitchRad;
			}
			
			float getYawRad() const {
				return yawRad;
			}
			
			const Vector3F& getPositionM() const {
				return positionM;
			}
			
			const Vector3F& getVelocityMs() const {
				return velocityMs;
			}
			
			const Vector3F& getAccelerationG() const {
				return _accelerationG;
			}

			void setCalibrationMode() {
				_MPU.setGyroRange(MPU9250_GYRO_RANGE_250);
				_MPU.setAccelRange(MPU9250_ACC_RANGE_2G);

				_MPU.setAccelDLPF(MPU9250_DLPF_6);
				_MPU.enableAccelDLPF();

				_MPU.setGyroDLPF(MPU9250_DLPF_6);
				_MPU.enableGyroDLPF();

				vTaskDelay(pdMS_TO_TICKS(100));

				_MPU.disableFIFO();
			}

			void setOperationalMode() {
				// Range
				_MPU.setAccelRange(MPU9250_ACC_RANGE_2G);
				_MPU.setGyroRange(MPU9250_GYRO_RANGE_250);

				// LPF
				_MPU.setAccelDLPF(MPU9250_DLPF_2);
				_MPU.enableAccelDLPF();

				_MPU.setGyroDLPF(MPU9250_DLPF_2);
				_MPU.enableGyroDLPF();

				vTaskDelay(pdMS_TO_TICKS(100));

				// FIFO
				_MPU.setFIFOMode(MPU9250_STOP_WHEN_FULL);
				_MPU.enableFIFO();

				// In some cases a delay after enabling FIFO makes sense
				vTaskDelay(pdMS_TO_TICKS(100));

				_MPU.setFIFODataSource(FIFODataSource);
			}

			Vector3F getRawAccelData() const {
				float x, y, z;
				_MPU.getAccelData(x, y, z);

				return { x, y, z };
			}

			Vector3F getRawGyroData() const {
				float x, y, z;
				_MPU.getGyroData(x, y, z);

				return { x, y, z };
			}

			Vector3F getRawMagData() const {
				float x, y, z;
				_MPU.getMagData(x, y, z);

				return { x, y, z };
			}

			const Vector3F& getAccelBias() const {
				return _accelBias;
			}

			void setAccelBias(const Vector3F& value) {
				_accelBias = value;

				ESP_LOGI(_logTag, "acc bias: %f x %f x %f", _accelBias.getX(), _accelBias.getY(), _accelBias.getZ());
			}

			const Vector3F& getGyroBias() const {
				return _gyroBias;
			}

			void setGyroBias(const Vector3F& value) {
				_gyroBias = value;

				ESP_LOGI(_logTag, "gyro bias: %f x %f x %f", _gyroBias.getX(), _gyroBias.getY(), _gyroBias.getZ());
			}

			const Vector3F& getMagBias() const {
				return _magBias;
			}

			void setMagBias(const Vector3F& value) {
				_magBias = value;

				ESP_LOGI(_logTag, "mag bias: %f x %f x %f", _magBias.getX(), _magBias.getY(), _magBias.getZ());
			}

		private:
			constexpr static auto _logTag = "IMU";
			
			// -------------------------------- MPU --------------------------------
			
			MPU9250 _MPU {};

			// -------------------------------- Accel --------------------------------
			
			Vector3F _accelBias {};
			Vector3F _accelerationG {};
			
			// -------------------------------- Gyro --------------------------------
			
			Vector3F _gyroBias {};
			
			// -------------------------------- Mag --------------------------------
			
			constexpr static float magLPFFactorPerSecond = 10.f;
			constexpr static float magLPFFactor = magLPFFactorPerSecond * static_cast<float>(magSampleIntervalUs) / 1'000'000.f;
			
			Vector3F _magBias {};
			Vector3F _magDataFiltered {};
			uint32_t _magSampleTimeUs = 0;

			void magTick() {
				if (esp_timer_get_time() < _magSampleTimeUs)
					return;

				float x, y, z;
				_MPU.getMagData(x, y, z);

//					ESP_LOGI(_logTag, "mag raw: %f x %f x %f", magData.getX(), magData.getY(), magData.getZ());
				
				// Axis swap, fuck MPU
				_magDataFiltered.setX(LowPassFilter::apply(_magDataFiltered.getX(), y - _magBias.getY(), magLPFFactor));
				_magDataFiltered.setY(LowPassFilter::apply(_magDataFiltered.getY(), x - _magBias.getX(), magLPFFactor));
				_magDataFiltered.setZ(LowPassFilter::apply(_magDataFiltered.getZ(), -(z - _magBias.getZ()), magLPFFactor));

//				magSample.setX(magData.getY() - magBias.getY());
//				magSample.setY(magData.getX() - magBias.getX());
//				magSample.setZ(-(magData.getZ() - magBias.getZ()));

//				const auto magYaw = std::atan2(magSample.getX(), magSample.getY());
//				ESP_LOGI(_logTag, "Mag: %f x %f x %f, yaw: %f", magData.getX(), magSample.getY(), magSample.getZ(), toDegrees(magYaw));
				
				_magSampleTimeUs = esp_timer_get_time() + magSampleIntervalUs;
			}
			
			// -------------------------------- FIFO --------------------------------
			
			constexpr static uint16_t FIFOLength = 512;
			constexpr static float FIFOSampleIntervalS = 1.0f / MPUSampleRateHz;
			
			constexpr static MPU9250_fifo_data_source FIFODataSource = MPU9250_FIFO_DATA_SOURCE_ACCEL_GYRO;
			// 3 axis * 2 bytes
			constexpr static uint8_t FIFOSampleDataTypeLength = 3 * 2;
			// Aacc + gyro
			constexpr static uint8_t FIFOSampleDataTypes = 2;
			constexpr static uint8_t FIFOSampleLength = FIFOSampleDataTypeLength * FIFOSampleDataTypes;
			constexpr static uint16_t FIFOMaxSampleCount = FIFOLength / FIFOSampleLength;
			
			constexpr static uint32_t FIFOBytesPerSecond = FIFOSampleLength * MPUSampleRateHz;
			constexpr static uint32_t FIFOMinimumSampleIntervalUs = FIFOLength * 1'000'000 / FIFOBytesPerSecond;
			constexpr static uint32_t FIFOSafeSampleIntervalUs = FIFOMinimumSampleIntervalUs * 9 / 10;
			
			uint32_t FIFOSampleTimeUs = 0;
			
			void FIFOTick() {
				if (esp_timer_get_time() < FIFOSampleTimeUs)
					return;
				
				const auto sampleCount = _MPU.getFIFOCount() / FIFOSampleLength;

//				ESP_LOGI(_logTag, "FIFO sample count %d", sampleCount);
				
				if (sampleCount < 8) {
					ESP_LOGW(_logTag, "FIFO sample count %d is not enough, skipping for more data", sampleCount);
					return;
				}
				else if (sampleCount >= FIFOMaxSampleCount) {
					ESP_LOGW(_logTag, "FIFO sample count %d exceeds max sample count, data was permanently lost", sampleCount);
				}
				
				_MPU.setFIFODataSource(MPU9250_FIFO_DATA_SOURCE_NONE);

//				ESP_LOGI(_logTag, "FIFO sample count: %d", sampleCount);
				
				const auto samplesToRead = std::min<uint16_t>(FIFOMaxSampleCount, sampleCount);
				
				uint8_t sample[FIFOSampleLength] {};
				Vector3F accelerationGSum {};
				float x, y, z;
				
				for (uint32_t i = 0; i < samplesToRead; i++) {
					// FIFO
					_MPU.getFIFOData(sample, FIFOSampleLength);

					// Accel
					_MPU.getAccelData(sample, x, y, z);
					const auto accelData = Vector3F(x, y, z) - _accelBias;
					accelerationGSum += accelData;

					// Gyro
					_MPU.getGyroData(sample + FIFOSampleDataTypeLength, x, y, z);
					const auto gyroData = Vector3F(x, y, z) - _gyroBias;
					
					// Applying adaptive complimentary filter
					AdaptiveComplimentaryFiler::apply(
						accelData,
						gyroData,
						_magDataFiltered,
						
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
					velocityMs += accelerationMs2 * FIFOSampleIntervalS;
					
					auto accelPositionOffsetM = velocityMs * FIFOSampleIntervalS;
					positionM += accelPositionOffsetM;
				}

//				ESP_LOGI(_logTag, "Velocity: %f x %f x %f", accelVelocityMs.getX(), accelVelocityMs.getY(), accelVelocityMs.getZ());
//				ESP_LOGI(_logTag, "Pos: %f x %f x %f", accelPosM.getX(), accelPosM.getY(), accelPosM.getZ());
//				ESP_LOGI(_logTag, "Roll pitch yaw: %f x %f x %f", toDegrees(rollRad), toDegrees(pitchRad), toDegrees(yawRad));
				
				_MPU.resetFIFO();
				_MPU.setFIFODataSource(FIFODataSource);
				_MPU.readAndClearInterruptStatus();
				
				accelerationGSum /= samplesToRead;
				_accelerationG = accelerationGSum;
				
				FIFOSampleTimeUs = esp_timer_get_time() + FIFOSafeSampleIntervalUs;
			}
			
			// -------------------------------- Computed --------------------------------
			
			float rollRad = 0;
			float pitchRad = 0;
			float yawRad = 0;
			
			Vector3F positionM {};
			Vector3F velocityMs {};
			
		
		public:
			constexpr static uint8_t accelerationGMax = 2;
			constexpr static uint32_t recommendedTickDelayUs = std::min(FIFOSafeSampleIntervalUs, magSampleIntervalUs);
	};
}