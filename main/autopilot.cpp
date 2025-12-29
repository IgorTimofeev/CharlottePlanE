#include "autopilot.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

#include "config.h"
#include "aircraft.h"
#include "utils/math.h"
#include "utils/lowPassFilter.h"

namespace pizda {
	void Autopilot::setup() {
		
	}
	
	void Autopilot::start() {
		xTaskCreate(
			[](void* arg) {
				reinterpret_cast<Autopilot*>(arg)->taskBody();
			},
			"Autopilot",
			4096,
			this,
			16,
			nullptr
		);
	}
	
	[[noreturn]] void Autopilot::taskBody() {
		auto& ac = Aircraft::getInstance();
		
		while (true) {
			const float altitudeDeltaM = static_cast<float>(ac.remoteData.raw.autopilot.altitudeM) - ac.ahrs.getAltitudeM();
			const bool climbUp = altitudeDeltaM >= 0;
			
			const float headingDeltaDeg = static_cast<float>(ac.remoteData.raw.autopilot.headingDeg) - ac.ahrs.getHeadingDeg();
			const bool rollRight = headingDeltaDeg >= 0;
			
			const float LPFFactor = 3 * static_cast<float>(_tickIntervalUs) / 1'000'000.f;
			
			ESP_LOGI(_logTag, "altitudeDeltaM: %f, headingDeltaDeg: %f", altitudeDeltaM, headingDeltaDeg);
			
			// Roll
			ac.aircraftData.computed.autopilotFlightDirectorRollRad = LowPassFilter::applyForAngleRad(
				ac.aircraftData.computed.autopilotFlightDirectorRollRad,
				rollRight
					? config::limits::autopilotRollAngleMaxRad
					: -config::limits::autopilotRollAngleMaxRad,
				LPFFactor
			);
			
			// Pitch
			ac.aircraftData.computed.autopilotFlightDirectorPitchRad = LowPassFilter::applyForAngleRad(
				ac.aircraftData.computed.autopilotFlightDirectorPitchRad,
				climbUp
					? config::limits::autopilotPitchAngleMaxRad
					: -config::limits::autopilotPitchAngleMaxRad,
				LPFFactor
			);
			
			ESP_LOGI(_logTag, "roll: %f, pitch: %f", toDegrees(ac.aircraftData.computed.autopilotFlightDirectorRollRad), toDegrees(ac.aircraftData.computed.autopilotFlightDirectorPitchRad));
			
			vTaskDelay(pdMS_TO_TICKS(_tickIntervalUs / 1'000));
		}
	}
}