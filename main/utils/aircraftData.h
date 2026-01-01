#pragma once

#include <cmath>

#include "utils/geographicCoordinates.h"

namespace pizda {
	class AircraftDataRaw {
		public:
		
	};
	
	class AircraftDataComputed {
		public:
			
	};
	
	class AircraftData {
		public:
			AircraftDataRaw raw {};
			AircraftDataComputed computed {};
	};
}