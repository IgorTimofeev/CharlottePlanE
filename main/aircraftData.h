#pragma once

#include <cmath>

#include "types.h"

namespace pizda {
	class AircraftDataCalibration {
		public:
			bool inProgress = false;
			AircraftCalibrationSystem system = AircraftCalibrationSystem::accelAndGyro;
			uint8_t progress = 0xFF;
	};
	
	class AircraftData {
		public:
			AircraftDataCalibration calibration {};
	};
}