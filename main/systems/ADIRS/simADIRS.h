#pragma once

#ifdef SIM

#include <cstring>
#include <array>
#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "systems/ADIRS/ADIRS.h"
#include "systems/simLink/simLink.h"

#include "types/generic.h"
#include "config.h"

namespace pizda {
	class SimADIRS : public ADIRS {
		protected:
			void onTick() override;
			void onCalibrateAccelAndGyro() override;
			void onCalibrateMag() override;
			
		private:
			static void simulateCalibration();
	};
}

#endif