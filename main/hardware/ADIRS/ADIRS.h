#pragma once

#include <array>
#include <cmath>

#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "utils/vector3.h"
#include "utils/geographicCoordinates.h"

namespace pizda {
	class ADIRS {
		public:
			uint32_t getReferencePressurePa() const {
				return _referencePressurePa;
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
			
			float getHeadingDeg() const {
				return _headingDeg;
			}
			
//			int32_t _speedTime = 0;
//			float _speed = 0;
			
			float getSlipAndSkidFactor() const {
				return _slipAndSkidFactor;
			}
			
			const GeographicCoordinates& getCoordinates() {
				return _coordinates;
			}
			
			float getAccelSpeedMPS() {
//				if (esp_timer_get_time() > _speedTime) {
//					_speed = random(14, 15);
//
//					_speedTime = esp_timer_get_time() + 1'000'000;
//				}
//
//				return _speed;
				
				return _accelSpeedMPS;
			}
			
			void setRollRad(float rollRad) {
				_rollRad = rollRad;
			}
			
			void setPitchRad(float pitchRad) {
				_pitchRad = pitchRad;
			}
			
			void setYawRad(float value) {
				_yawRad = value;
			}
			
			void updateHeadingFromYaw() {
				_headingDeg = toDegrees(-_yawRad);
			}
			
			void setAccelSpeedMPS(float accelSpeedMPS) {
				_accelSpeedMPS = accelSpeedMPS;
			}
			
			static float computeAltitude(
				float pressurePa,
				float temperatureC,
				uint32_t referencePressurePa,
				float lapseRateKpm = -0.0065f
			) {
				// Physical constants
				constexpr static float g = 9.80665f;       // Gravitational acceleration (m/s²)
				constexpr static float R = 8.314462618f;   // Universal gas constant (J/(mol·K))
				constexpr static float M = 0.0289644f;     // Molar mass of dry air (kg/mol)
				
				// Convert temperature from Celsius to Kelvin
				const float temperatureK = temperatureC + 273.15f;
				
				// Avoid division by zero and invalid values
				if (pressurePa <= 0.0f || temperatureK <= 0.0f)
					return 0.0f;
				
				// Barometric formula with temperature gradient consideration
				// Using International Standard Atmosphere (ISA) model
				// h = (T0 / L) * (1 - (P / P0)^(R * L / (g * M)))
				
				// If temperature lapse rate is close to zero, use simplified formula
				if (std::abs(lapseRateKpm) < 1e-6f) {
					// Isothermal atmosphere (lapse rate ≈ 0)
					return (R * temperatureK) / (g * M) * std::log(static_cast<float>(referencePressurePa) / pressurePa);
				}
				
				// Full formula with temperature gradient
				const float exponent = (R * lapseRateKpm) / (g * M);
				const float power = std::pow(pressurePa / static_cast<float>(referencePressurePa), exponent);
				const float altitude = (temperatureK / lapseRateKpm) * (1.0f - power);
				
				return altitude;
			}
			
			void updateSlipAndSkidFactor(float lateralAcceleration, float GMax) {
				_slipAndSkidFactor =
					std::clamp<float>(-lateralAcceleration - std::sin(getRollRad()), -GMax, GMax)
					/ static_cast<float>(GMax);
			}
			
			void setPressurePa(float pressurePa) {
				_pressurePa = pressurePa;
			}
			
			void setTemperatureC(float temperatureC) {
				_temperatureC = temperatureC;
			}
			
			void setReferencePressurePa(uint32_t value) {
				_referencePressurePa = value;
			}
			
			void updateAltitudeFromPressureTemperatureAndReferenceValue() {
				_coordinates.setAltitude(computeAltitude(_pressurePa, _temperatureC, _referencePressurePa));
			}
			
			void setLatitude(float value) {
				_coordinates.setLatitude(value);
			}
			
			void setLongitude(float value) {
				_coordinates.setLongitude(value);
			}
			
		protected:
			constexpr static const char* _logTag = "ADIRS";
			
		private:
			float _rollRad = 0;
			float _pitchRad = 0;
			
			float _yawRad = 0;
			float _headingDeg = 0;
			
			float _accelSpeedMPS = 0;
			float _slipAndSkidFactor = 0;
			
			float _pressurePa = 0;
			float _temperatureC = 0;
			
			uint32_t _referencePressurePa = 101325;
			
			// 60.014002019765776, 29.717151511256816
			// ОПЯТЬ ЖЕНЩИНЫ??? ФЕДЯ СУКА ЭТО ТЫ ЕБЛАН СДЕЛАЛ
			GeographicCoordinates _coordinates = {
				toRadians(60.014581566191914f),
				toRadians(29.70258579817704f),
				0
			};
	};
}