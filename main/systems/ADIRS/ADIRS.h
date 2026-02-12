#pragma once

#include <esp_log.h>

#include <geoCoordinates.h>

#include "utilities/math.h"

namespace pizda {
	using namespace YOBA;

	class ADIRS {
		public:
			virtual ~ADIRS() = default;

			virtual void setup();
			void setupAsync();

			float getRollRad() const;
			float getPitchRad() const;
			float getYawRad() const;
			float getHeadingDeg() const;

			//			int32_t _speedTime = 0;
//			float _speed = 0;
			
			float getSlipAndSkidFactor() const;
			const GeoCoordinates& getCoordinates() const;
			float getAccelSpeedMPS() const;
			void setReferencePressurePa(const uint32_t value);

		protected:
			constexpr static auto _logTag = "ADIRS";
			
			virtual void onTick() = 0;
			virtual void onCalibrateAccelAndGyro() = 0;
			virtual void onCalibrateMag() = 0;

			void setRollRad(const float rollRad);
			void setPitchRad(const float pitchRad);
			void setYawRad(const float value);
			void updateHeadingFromYaw();
			void setAccelSpeedMPS(const float accelSpeedMPS);

			static float computeAltitude(
				const float pressurePa,
				const float temperatureC,
				const uint32_t referencePressurePa,
				const float lapseRateKpm = -0.0065f
			);

			void updateSlipAndSkidFactor(const float lateralAcceleration, const float GMax);
			void setPressurePa(const float pressurePa);
			void setTemperatureC(const float temperatureC);
			void updateAltitudeFromPressureTemperatureAndReferenceValue();
			void setLatitude(const float value);
			void setLongitude(const float value);

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
			GeoCoordinates _coordinates {
				toRadians(60.014581566191914f),
				toRadians(29.70258579817704f),
				0
			};
			
			[[noreturn]] void onStart();
	};
}