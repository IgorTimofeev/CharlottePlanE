#pragma once

#include <cstdint>
#include <algorithm>

namespace YOBA {
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
			bool reverse = false;
			
			void sanitize() {
				min = std::clamp<uint16_t>(min, 0, 3000);
				max = std::clamp<uint16_t>(max, 0, 3000);
				
				if (min > max)
					std::swap(min, max);
			}
	};
}