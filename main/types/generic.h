#pragma once

#include <cstdint>
#include <algorithm>

namespace pizda {
	enum class ConnectionState : uint8_t {
		initial,
		connected,
		disconnected
	};
	
	enum class AircraftCalibrationSystem : uint8_t {
		accelAndGyro,
		mag
	};
	
	enum class AutopilotLateralMode : uint8_t {
		man,
		hdg
	};
	
	enum class AutopilotVerticalMode : uint8_t {
		man,
		flc,
		alts,
		alt
	};
	
	class MotorConfiguration {
		public:
			uint16_t min = 1000;
			uint16_t max = 2000;
			uint16_t startup = 1500;
			int16_t offset = 0;
			bool reverse = false;
			
			void sanitize() {
				min = std::clamp<uint16_t>(min, 100, 1400);
				max = std::clamp<uint16_t>(max, 1600, 2900);
				
				if (min > max)
					std::swap(min, max);
				
				startup = std::clamp<uint16_t>(startup, min, max);
				
				if (std::abs(offset) > 900)
					offset = 0;
			}
	};
}