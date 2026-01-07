#pragma once

#include <cstdint>

namespace pizda {
	enum class ConnectionState : uint8_t {
		initial,
		connected,
		disconnected
	};
	
	enum class RemoteState : uint8_t {
		normal
	};
	
	enum class AircraftState : uint8_t {
		aircraftNormal,
		aircraftCalibrating
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