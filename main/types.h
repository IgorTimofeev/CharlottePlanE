#pragma once

#include <cstdint>

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
}