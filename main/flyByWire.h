#pragma once

#include <cstdint>

namespace pizda {
	class FlyByWire {
		public:
			void setup();
			void start();
			
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
			
			static float getInterpolationFactor(float range, float rangeMax);
			static float predictValue(float valueDelta, uint32_t dueTimeUs, uint32_t deltaTimeUs);
			
			void computeData();
			[[noreturn]] void taskBody();
			
	};
}