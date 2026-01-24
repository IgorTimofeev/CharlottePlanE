#pragma once

#include <cstring>
#include <array>
#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "hardware/ADIRS/ADIRS.h"
#include "hardware/simLink/simLink.h"

#include "core/generic.h"
#include "config.h"

namespace pizda {
	class SimADIRS : public ADIRS {
		public:
			void setup();
			
		protected:
			void onTick() override;
			void onCalibrateAccelAndGyro() override;
			void onCalibrateMag() override;
			
		private:
			void simulateCalibration();
	};
}