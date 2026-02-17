#pragma once

#include <cmath>

#include "types/generic.h"

namespace pizda {
	class AircraftDataCalibration {
		public:
			bool calibrating = false;
			AircraftCalibrationSystem system = AircraftCalibrationSystem::accelAndGyro;
			uint8_t progress = 0xFF;
	};

	class AircraftDataTransceiver {
		public:
			TransceiverCommunicationSettings receivedCommunicationSettings {};
	};
	
	class AircraftData {
		public:
			AircraftDataCalibration calibration {};
			AircraftDataTransceiver transceiver {};
	};
}