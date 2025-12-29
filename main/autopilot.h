#pragma once

#include <cstdint>

namespace pizda {
	class Autopilot {
		public:
			void setup();
			void start();
			
		private:
			constexpr static const char* _logTag = "Aircraft";
			
			constexpr static uint32_t _tickFrequencyHz = 30;
			constexpr static uint32_t _tickIntervalUs = 1'000'000 / _tickFrequencyHz;
			
			[[noreturn]] void taskBody();
			
			static float getInterpolatedLPFFactor(float value, float valueRange, float factorPerSecondMin, float factorPerSecondMax, uint32_t deltaTimeUs);
			static float predictValue(float valueDelta, uint32_t dueTimeUs, uint32_t deltaTimeUs);
	};
}