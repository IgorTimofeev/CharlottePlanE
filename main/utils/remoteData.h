#pragma once

#include <cmath>

namespace pizda {
	class RemoteDataRawAutopilot {
		public:
			uint8_t speedMPS = 0;
			bool autoThrottle = false;
			
			uint16_t headingDeg = 0;
			bool headingHold = false;
			
			uint16_t altitudeM = 0;
			bool levelChange = false;
			
			bool engaged = false;
	};
	
	class RemoteDataRaw {
		public:
			RemoteDataRawAutopilot autopilot {};
	};
	
	class RemoteDataComputed {
		public:
		
	};
	
	class RemoteData {
		public:
			RemoteDataRaw raw {};
			RemoteDataComputed computed {};
	};
}