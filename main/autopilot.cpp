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
	
	float Autopilot::getInterpolatedLPFFactor(
		float value,
		float valueRange,
		
		float factorPerSecondMin,
		float factorPerSecondMax,
		
		uint32_t deltaTimeUs
	) {
		const float factorPerSecondFactor = std::min(std::abs(value), valueRange) / valueRange;
		
		return LowPassFilter::getFactor(
			factorPerSecondMin + (factorPerSecondMax - factorPerSecondMin) * factorPerSecondFactor,
			deltaTimeUs
		);
	}
	
	float Autopilot::predictValue(float valueDelta, uint32_t deltaTimeUs, uint32_t dueTimeUs) {
		// valueDelta - deltaTimeUs
		// x          - dueTimeUs
		
		return valueDelta * static_cast<float>(dueTimeUs) / static_cast<float>(deltaTimeUs);
	}
	
	[[noreturn]] void Autopilot::taskBody() {
		auto& ac = Aircraft::getInstance();
		
		int64_t timeUs = esp_timer_get_time();
		
		float altitudePrevM = ac.ahrs.getAltitudeM();
		float headingPrevDeg = ac.ahrs.getHeadingDeg();
		
		while (true) {
			const auto deltaTimeUs = esp_timer_get_time() - timeUs;
			timeUs = esp_timer_get_time();
			
			// Altitude
			const auto altitudeM = ac.ahrs.getAltitudeM();
			
			const auto altitudePrevDeltaM = altitudeM - altitudePrevM;
			const auto altitudePredictedM = altitudePrevM + predictValue(altitudePrevDeltaM, deltaTimeUs, 1'000'000);
			altitudePrevM = altitudeM;
			
			const float altitudePredictedTargetDeltaM = static_cast<float>(ac.remoteData.raw.autopilot.altitudeM) - altitudePredictedM;
			const bool climbUp = altitudePredictedTargetDeltaM >= 0;
			
			// Heading
			const auto headingDeg = ac.ahrs.getHeadingDeg();
			
			auto headingPrevDeltaDeg = headingDeg - headingPrevDeg;
			
			if (headingPrevDeltaDeg > 180) {
				headingPrevDeltaDeg -= 360;
			}
			else if (headingPrevDeltaDeg < -180) {
				headingPrevDeltaDeg += 360;
			}
			
			const auto headingPredictedDeg = headingPrevDeg + predictValue(headingPrevDeltaDeg, deltaTimeUs, 1'000'000);
			headingPrevDeg = headingDeg;
			
			const float headingPredictedTargetDeltaDeg = static_cast<float>(ac.remoteData.raw.autopilot.headingDeg) - headingPredictedDeg;
		
//			ESP_LOGI(_logTag, "altitudeDeltaM: %f, headingDeltaDeg: %f", altitudeDeltaM, headingDeltaDeg);

			// Pitch
			auto LPFFactor = getInterpolatedLPFFactor(
				altitudePredictedTargetDeltaM, 30,
				0.05, 0.9,
				deltaTimeUs
			);
			
			const auto pitchAngle =
				climbUp
				? config::limits::autopilotPitchAngleMaxRad
				: -config::limits::autopilotPitchAngleMaxRad;
			
			ac.aircraftData.computed.autopilotPitchRad = LowPassFilter::applyForAngleRad(
				ac.aircraftData.computed.autopilotPitchRad,
				pitchAngle,
				LPFFactor
			);
			
			ac.aircraftData.computed.autopilotFlightDirectorPitchRad = ac.aircraftData.computed.autopilotPitchRad - ac.ahrs.getPitchRad();
			
			// Turn
			const bool turnToRight = headingPredictedTargetDeltaDeg >= 0;
			
			const auto rollAngle =
				turnToRight
				? config::limits::autopilotRollAngleMaxRad
				: -config::limits::autopilotRollAngleMaxRad;
			
			LPFFactor = getInterpolatedLPFFactor(
				headingPredictedTargetDeltaDeg, 45,
				0.05, 0.9,
				deltaTimeUs
			);
			
			ac.aircraftData.computed.autopilotRollRad = LowPassFilter::applyForAngleRad(
				ac.aircraftData.computed.autopilotRollRad,
				rollAngle,
				LPFFactor
			);
			
			ac.aircraftData.computed.autopilotFlightDirectorRollRad = ac.aircraftData.computed.autopilotRollRad - ac.ahrs.getRollRad();
			
			vTaskDelay(pdMS_TO_TICKS(_tickIntervalUs / 1'000));
		}
	}
}