#pragma once

#include "hardware/ADIRS/ADIRS.h"

#include <cstring>
#include <array>
#include <cmath>

#include <esp_log.h>

#include "config.h"

namespace pizda {
	class DummyADIRS : public ADIRS {
		public:
			void setup() {
			
			}
	};
}