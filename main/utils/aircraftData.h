#pragma once

#include <cmath>

namespace pizda {
	class AircraftDataRaw {
		public:
		
	};
	
	class AircraftDataComputed {
		public:
			float autopilotFlightDirectorRollRad = 0;
			float autopilotFlightDirectorPitchRad = 0;
	};
	
	class AircraftData {
		public:
			AircraftDataRaw raw {};
			AircraftDataComputed computed {};
	};
}