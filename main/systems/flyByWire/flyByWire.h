#pragma once

#include <cstdint>

#include <PIDController.h>

#include "types/generic.h"

namespace pizda {
	using namespace YOBA;

	class FlyByWire {
		public:
			void setup();
			
			float getSelectedSpeedMps() const;
			void setSelectedSpeedMps(float value);
			
			uint16_t getSelectedHeadingDeg() const;
			void setSelectedHeadingDeg(uint16_t value);
			
			float getSelectedAltitudeM() const;
			void setSelectedAltitudeM(float value);
			
			AutopilotLateralMode getLateralMode() const;
			void setLateralMode(AutopilotLateralMode value);
			
			AutopilotVerticalMode getVerticalMode() const;
			void setVerticalMode(AutopilotVerticalMode value);
			
			bool getAutothrottle() const;
			void setAutothrottle(bool value);
			
			bool getAutopilot() const;
			void setAutopilot(bool value);

			bool getGyro() const;
			void setGyro(bool value);

			float getTargetRollRad() const;
			float getTargetPitchRad() const;

		private:
			constexpr static auto _logTag = "FlyByWire";
			
			constexpr static uint32_t _tickFrequencyHz = 30;

			int64_t _computationTimeUs = 0;
			
			float _speedPrevMPS = 0;
			float _altitudePrevM = 0;
			float _rollPrevRad = 0;
			float _pitchPrevRad = 0;
			float _yawPrevRad = 0;
			
			float _throttleTargetFactor = 0;
			float _rollTargetRad = 0;
			float _pitchTargetRad = 0;
			
			float _aileronsFactor = 0.5;
			float _elevatorFactor = 0.5;
			float _rudderFactor = 0.5;

			float _speedSelectedMPS = 0;
			uint16_t _headingSelectedDeg = 0;
			float _altitudeSelectedM = 0;

			AutopilotLateralMode _lateralMode = AutopilotLateralMode::dir;
			AutopilotVerticalMode _verticalMode = AutopilotVerticalMode::dir;

			PIDController _targetToRollPID {};
			PIDController _targetToPitchPID {};
			PIDController _rollToAileronsPID {};
			PIDController _pitchToElevatorPID {};

			bool _autothrottle = false;
			bool _autopilot = false;

			static float mapPizda(float min, float max, float factor);
			static float getInterpolationFactor(float range, float rangeMax);
			static float predictValue(float valueDelta, float deltaTimeS, float dueTimeS);
			
			void computeData();
			void applyData() const;

			[[noreturn]] void onStart();
			
	};
}