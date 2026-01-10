#pragma once

#include <cstdint>

#include "types/generic.h"

namespace pizda {
	
	
	class FlyByWire {
		public:
			void setup();
			void start();
			
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
			
			float getTargetRollRad() const;
			float getTargetPitchRad() const;
			
			void applyData();
		
		private:
			constexpr static const char* _logTag = "Aircraft";
			
			constexpr static uint32_t _tickFrequencyHz = 20;
			constexpr static uint32_t _tickIntervalUs = 1'000'000 / _tickFrequencyHz;
			
			int64_t _computationTimeUs = 0;
			
			float _speedPrevMPS = 0;
			float _altitudePrevM = 0;
			float _rollPrevRad = 0;
			float _pitchPrevRad = 0;
			float _yawPrevRad = 0;
			
			float _throttleTargetFactor = 0;
			float _rollTargetRad = 0;
			float _pitchTargetRad = 0;
			
			float _aileronsTargetFactor = 0.5;
			float _elevatorTargetFactor = 0.5;
			
			float _speedSelectedMPS = 0;
			uint16_t _headingSelectedDeg = 0;
			float _altitudeSelectedM = 0;
			
			AutopilotLateralMode _lateralMode = AutopilotLateralMode::man;
			AutopilotVerticalMode _verticalMode = AutopilotVerticalMode::man;
			
			bool _autothrottle = false;
			bool _autopilot = false;
			
			static float mapPizda(float min, float max, float factor);
			static float getInterpolationFactor(float range, float rangeMax);
			static float predictValue(float valueDelta, uint32_t dueTimeUs, uint32_t deltaTimeUs);
			
			void computeData();
			[[noreturn]] void taskBody();
			
	};
}