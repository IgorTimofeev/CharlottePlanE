#pragma once

#include <cmath>

#include "types/generic.h"

namespace pizda {
	class RemoteDataRawControls {
		public:
			// Factor in [0.0; 1.0] range
			float throttle = 0;
			float ailerons = 0;
			float elevator = 0;
			float rudder = 0;
			float flaps = 0;
			float unused = 0;
	};
	
	class RemoteDataRaw {
		public:
			RemoteDataRawControls controls {};
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