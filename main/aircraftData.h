#pragma once

#include <cmath>

#include "types.h"

namespace pizda {
	class AircraftDataCalibration {
		public:
			AircraftCalibrationSystem system = AircraftCalibrationSystem::accelAndGyro;
			uint8_t progress = 0;
	};
	
	class AircraftData {
		public:
			AircraftState state = AircraftState::normal;
			AircraftDataCalibration calibration {};
	};
}