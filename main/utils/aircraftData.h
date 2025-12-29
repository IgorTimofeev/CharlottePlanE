#pragma once

#include <cmath>

namespace pizda {
	class AircraftDataRaw {
		public:
		
	};
	
	class AircraftDataComputed {
		public:
			float autopilotRollRad = 0;
			float autopilotPitchRad = 0;
			
			float autopilotFlightDirectorPitchRad = 0;
			float autopilotFlightDirectorRollRad = 0;
	};
	
	class AircraftData {
		public:
			AircraftDataRaw raw {};
			AircraftDataComputed computed {};
	};
}