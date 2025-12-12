#pragma once

#include "constants.h"
#include "hardware/ahrs/gy-91.h"

namespace pizda {
	class AHRS {
		public:
			void setup() {
				_adiru1.setup();
			}

		private:
			GY91 _adiru1 {
				constants::adiru1::mpu9250ss,
				constants::adiru1::bmp280ss
			};
	};
}