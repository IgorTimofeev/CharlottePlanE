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
		float rollPrevRad = ac.ahrs.getRollRad();
		float pitchPrevRad = ac.ahrs.getPitchRad();
		float yawPrevRad = ac.ahrs.getYawRad();
		
		while (true) {
			const auto deltaTimeUs = esp_timer_get_time() - timeUs;
			timeUs = esp_timer_get_time();
			
			// -------------------------------- Prediction --------------------------------
			
			constexpr static uint32_t predictionTimeUs = 1'000'000;
			
			// Altitude
			const auto altitudeM = ac.ahrs.getAltitudeM();
			const auto altitudePrevDeltaM = altitudeM - altitudePrevM;
			const auto altitudePredictedM = altitudePrevM + predictValue(altitudePrevDeltaM, deltaTimeUs, predictionTimeUs);
			altitudePrevM = altitudeM;
			
			// Roll
			const auto rollRad = ac.ahrs.getRollRad();
			const auto rollPrevDeltaRad = rollRad - rollPrevRad;
			const auto rollPredictedRad = rollPrevRad + predictValue(rollPrevDeltaRad, deltaTimeUs, predictionTimeUs);
			rollPrevRad = rollRad;
			
			// Pitch
			const auto pitchRad = ac.ahrs.getPitchRad();
			const auto pitchPrevDeltaRad = pitchRad - pitchPrevRad;
			const auto pitchPredictedRad = pitchPrevRad + predictValue(pitchPrevDeltaRad, deltaTimeUs, predictionTimeUs);
			pitchPrevRad = pitchRad;
			
			// Yaw
			const auto yawRad = ac.ahrs.getYawRad();
			const auto yawPrevDeltaRad = yawRad - yawPrevRad;
			const auto yawPredictedRad = yawPrevRad + predictValue(yawPrevDeltaRad, deltaTimeUs, predictionTimeUs);
			yawPrevRad = yawRad;
		
//			ESP_LOGI(_logTag, "altitudeDeltaM: %f, headingDeltaDeg: %f", altitudeDeltaM, headingDeltaDeg);
			
			// -------------------------------- Pitch --------------------------------
			
			const auto altitudeTargetM = static_cast<float>(ac.remoteData.raw.autopilot.altitudeM);
			const auto altitudePredictedTargetDeltaM = altitudeTargetM - altitudePredictedM;
			const auto climbUp = altitudePredictedTargetDeltaM >= 0;
			
			const auto pitchAngle =
				climbUp
				? config::limits::autopilotPitchAngleMaxRad
				: -config::limits::autopilotPitchAngleMaxRad;
			
			ac.aircraftData.computed.autopilotPitchRad = LowPassFilter::applyForAngleRad(
				ac.aircraftData.computed.autopilotPitchRad,
				pitchAngle,
				getInterpolatedLPFFactor(
					altitudePredictedTargetDeltaM, toRadians(30),
					0.01, 1.0,
					deltaTimeUs
				)
			);
			
			// -------------------------------- Roll --------------------------------
			
			const auto yawTargetRad = -toRadians(normalizeAngle180(static_cast<float>(ac.remoteData.raw.autopilot.headingDeg)));
			const auto yawPredictedTargetDeltaRad = yawTargetRad - yawPredictedRad;
			
			const auto yawToRight =
				(yawPredictedTargetDeltaRad < 0 && yawPredictedTargetDeltaRad < std::numbers::pi_v<float>)
				|| (yawPredictedTargetDeltaRad > 0 && yawPredictedTargetDeltaRad < -std::numbers::pi_v<float>);
			
			const bool rollToRight =
				(yawToRight && rollPredictedRad < config::limits::autopilotRollAngleMaxRad)
				|| (!yawToRight && rollPredictedRad < -config::limits::autopilotRollAngleMaxRad);
			
			const auto rollTargetRad =
				rollToRight
				? config::limits::autopilotRollAngleMaxRad
				: -config::limits::autopilotRollAngleMaxRad;
			
			const auto rollPredictedTargetDeltaRad = rollTargetRad - rollPredictedRad;
			
			ac.aircraftData.computed.autopilotRollRad = LowPassFilter::applyForAngleRad(
				ac.aircraftData.computed.autopilotRollRad,
				rollTargetRad,
				getInterpolatedLPFFactor(
					yawPredictedTargetDeltaRad, toRadians(40),
					0.01, 1.0,
					deltaTimeUs
				)
			);
			
			vTaskDelay(pdMS_TO_TICKS(_tickIntervalUs / 1'000));
		}
	}
}