#include "flyByWire.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

#include "config.h"
#include "aircraft.h"
#include "utils/math.h"
#include "utils/lowPassFilter.h"

namespace pizda {
	void FlyByWire::setup() {
		
	}
	
	void FlyByWire::start() {
		
		xTaskCreate(
			[](void* arg) {
				reinterpret_cast<FlyByWire*>(arg)->taskBody();
			},
			"Autopilot",
			4096,
			this,
			16,
			nullptr
		);
	}
	
	float FlyByWire::getTargetRollRad() const {
		return _rollTargetRad;
	}
	
	float FlyByWire::getTargetPitchRad() const {
		return _pitchTargetRad;
	}
	
	float FlyByWire::getInterpolatedLPFFactor(
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
	
	float FlyByWire::predictValue(float valueDelta, uint32_t deltaTimeUs, uint32_t dueTimeUs) {
		// valueDelta - deltaTimeUs
		// x          - dueTimeUs
		
		return valueDelta * static_cast<float>(dueTimeUs) / static_cast<float>(deltaTimeUs);
	}
	
	void FlyByWire::computeData() {
		auto& ac = Aircraft::getInstance();
		
		const auto deltaTimeUs = esp_timer_get_time() - _computationTimeUs;
		_computationTimeUs = esp_timer_get_time();
		
		// -------------------------------- Prediction --------------------------------
		
		constexpr static uint32_t predictionTimeUs = 1'000'000;
		
		// Speed
		const auto speedMPS = ac.adirs.getAccelSpeedMPS();
		const auto speedPrevDeltaMPS = speedMPS - _speedPrevMPS;
		const auto speedPredictedMPS = _speedPrevMPS + predictValue(speedPrevDeltaMPS, deltaTimeUs, predictionTimeUs);
		_speedPrevMPS = speedMPS;
		
		// Altitude
		const auto altitudeM = ac.adirs.getCoordinates().getAltitude();
		const auto altitudePrevDeltaM = altitudeM - _altitudePrevM;
		const auto altitudePredictedM = _altitudePrevM + predictValue(altitudePrevDeltaM, deltaTimeUs, predictionTimeUs);
		_altitudePrevM = altitudeM;
		
		// Roll
		const auto rollRad = ac.adirs.getRollRad();
		const auto rollPrevDeltaRad = rollRad - _rollPrevRad;
		const auto rollPredictedRad = _rollPrevRad + predictValue(rollPrevDeltaRad, deltaTimeUs, 1'000'000);
		_rollPrevRad = rollRad;
		
		// Pitch
		const auto pitchRad = ac.adirs.getPitchRad();
		const auto pitchPrevDeltaRad = pitchRad - _pitchPrevRad;
		const auto pitchPredictedRad = _pitchPrevRad + predictValue(pitchPrevDeltaRad, deltaTimeUs, predictionTimeUs);
		_pitchPrevRad = pitchRad;
		
		// Yaw
		const auto yawRad = ac.adirs.getYawRad();
		const auto yawPrevDeltaRad = yawRad - _yawPrevRad;
		const auto yawPredictedRad = _yawPrevRad + predictValue(yawPrevDeltaRad, deltaTimeUs, 2'000'000);
		_yawPrevRad = yawRad;


//			ESP_LOGI(_logTag, "altitudeDeltaM: %f, headingDeltaDeg: %f", altitudeDeltaM, headingDeltaDeg);
		
		// -------------------------------- Target deltas --------------------------------
		
		const auto speedTargetMPS = ac.remoteData.raw.autopilot.speedMPS;
		const auto speedTargetAndPredictedDeltaMPS = speedTargetMPS - speedPredictedMPS;
		
		const auto altitudeTargetM = ac.remoteData.raw.autopilot.altitudeM;
		const auto altitudeTargetAndPredictedDeltaM = altitudeTargetM - altitudePredictedM;
		
		const auto yawTargetRad = -toRadians(normalizeAngle180(static_cast<float>(ac.remoteData.raw.autopilot.headingDeg)));
		const auto yawTargetAndPredictedDeltaRad = yawTargetRad - yawPredictedRad;
		
		// -------------------------------- Pitch --------------------------------
		
//		ESP_LOGI(_logTag, "altitudeTargetM: %f, altitudeM: %f, altitudePredictedM: %f, altitudeTargetAndPredictedDeltaM: %f",
//			altitudeTargetM,
//			altitudeM,
//			altitudePredictedM,
//			altitudeTargetAndPredictedDeltaM
//		);
		
		_pitchTargetRad =
			ac.remoteData.raw.autopilot.levelChange
			? LowPassFilter::applyForAngleRad(
				_pitchTargetRad,
				altitudeTargetAndPredictedDeltaM >= 0
					? config::limits::autopilotPitchAngleMaxRad
					: -config::limits::autopilotPitchAngleMaxRad,
				getInterpolatedLPFFactor(
					altitudeTargetAndPredictedDeltaM, 100,
					0.2, 1.5,
					deltaTimeUs
				)
			)
			: 0;
		
		// -------------------------------- Throttle --------------------------------
		
		const auto throttleTargetAltitudeSafetyMarginM = 10.f;
		
//		ESP_LOGI(_logTag, "speedMPS: %f, speedTargetMPS: %f, speedPredictedMPS: %f, speedTargetAndPredictedDeltaMPS: %f",
//			speedMPS,
//			speedTargetMPS,
//			speedPredictedMPS,
//			speedTargetAndPredictedDeltaMPS
//		);
		
		auto throttleState =
			// Not enough speed
			speedTargetAndPredictedDeltaMPS > 0
			// Enough, but
			|| (
				// Altitude affects throttle
				ac.remoteData.raw.autopilot.levelChange
				// Target altitude hasn't been reached yet
				&& altitudeTargetAndPredictedDeltaM > throttleTargetAltitudeSafetyMarginM
			);
			
		const auto throttleTargetFactorMax = 1.f;
		
		_throttleTargetFactor = LowPassFilter::applyForAngleRad(
			_throttleTargetFactor,
			throttleState ? throttleTargetFactorMax : 0.f,
			getInterpolatedLPFFactor(
				speedTargetAndPredictedDeltaMPS, 5,
				0.01, 0.8,
				deltaTimeUs
			)
		);
		
		// -------------------------------- Roll --------------------------------
		
		const auto yawToRight =
			(yawTargetAndPredictedDeltaRad < 0 && yawTargetAndPredictedDeltaRad < std::numbers::pi_v<float>)
			|| (yawTargetAndPredictedDeltaRad > 0 && yawTargetAndPredictedDeltaRad < -std::numbers::pi_v<float>);
		
		const bool rollToRight =
			(yawToRight && rollPredictedRad < config::limits::autopilotRollAngleMaxRad)
			|| (!yawToRight && rollPredictedRad < -config::limits::autopilotRollAngleMaxRad);
		
		const auto rollTargetRad =
			rollToRight
			? config::limits::autopilotRollAngleMaxRad
			: -config::limits::autopilotRollAngleMaxRad;
		
		_rollTargetRad =
			ac.remoteData.raw.autopilot.headingHold
			? LowPassFilter::applyForAngleRad(
				_rollTargetRad,
				rollTargetRad,
				getInterpolatedLPFFactor(
					yawTargetAndPredictedDeltaRad, toRadians(30),
					0.01, 0.8,
					deltaTimeUs
				)
			)
			: 0;
		
		// -------------------------------- Ailerons --------------------------------
		
		const auto rollTargetAndPredictedDeltaRad = _rollTargetRad - rollPredictedRad;
		
		_aileronsTargetFactor =
			ac.remoteData.raw.autopilot.headingHold
			? LowPassFilter::applyForAngleRad(
				_aileronsTargetFactor,
				rollTargetAndPredictedDeltaRad >= 0 ? 1 : 0,
				getInterpolatedLPFFactor(
					rollTargetAndPredictedDeltaRad, toRadians(30),
					0.05, 0.8,
					deltaTimeUs
				)
			)
			: 0;
		
		// -------------------------------- Elevator --------------------------------
		
		const auto pitchTargetAndPredictedDeltaRad = _pitchTargetRad - pitchPredictedRad;
		
		_elevatorTargetFactor =
			ac.remoteData.raw.autopilot.levelChange
			? LowPassFilter::applyForAngleRad(
				_elevatorTargetFactor,
				pitchTargetAndPredictedDeltaRad >= 0 ? 0 : 1,
				getInterpolatedLPFFactor(
					pitchTargetAndPredictedDeltaRad, config::limits::autopilotPitchAngleMaxRad,
					0.2, 1.5,
					deltaTimeUs
				)
			)
			: 0;
	}
	
	void FlyByWire::applyData() {
		auto& ac = Aircraft::getInstance();
		
		// Throttle
		{
			const auto channel = ac.channels.getUintChannel(ChannelType::throttle);
			const auto motor = ac.motors.getMotor(MotorType::throttle);
			
			if (!channel || !motor)
				return;
			
			motor->setPowerF(
				ac.remoteData.raw.autopilot.autoThrottle
				? _throttleTargetFactor
				: channel->getValueF()
			);
		}
		
		// Ailerons
		{
			const auto channel = ac.channels.getUintChannel(ChannelType::ailerons);
			const auto leftAileronMotor = ac.motors.getMotor(MotorType::leftAileron);
//				const auto rightAileronMotor = ac.motors.getMotor(MotorType::rightAileron);
			
			if (!channel || !leftAileronMotor)
				return;
			
			leftAileronMotor->setPowerF(
				ac.remoteData.raw.autopilot.engaged && ac.remoteData.raw.autopilot.headingHold
				? _aileronsTargetFactor
				: channel->getValueF()
			);
//				rightAileronMotor->setPower(aileronsChannel->getValue());
		}
		
		// Elevator & rudder
		{
			const auto elevatorChannel = ac.channels.getUintChannel(ChannelType::elevator);
			const auto rudderChannel = ac.channels.getUintChannel(ChannelType::rudder);
			
			const auto leftTailMotor = ac.motors.getMotor(MotorType::leftTail);
			const auto rightTailMotor = ac.motors.getMotor(MotorType::rightTail);
			
			if (!elevatorChannel || !rudderChannel || !leftTailMotor || !rightTailMotor)
				return;
			
			leftTailMotor->setPowerF(
				ac.remoteData.raw.autopilot.engaged && ac.remoteData.raw.autopilot.levelChange
				? _elevatorTargetFactor
				: elevatorChannel->getValueF()
			);
			
			rightTailMotor->setPowerF(rudderChannel->getValueF());
		}
		
		// Flaps
		{
			const auto flapsChannel = ac.channels.getUintChannel(ChannelType::flaps);
			const auto leftFlapMotor = ac.motors.getMotor(MotorType::leftFlap);
//				const auto rightFlapMotor = ac.motors.getMotor(MotorType::rightAileron);
			
			if (!flapsChannel || !leftFlapMotor)
				return;
			
			leftFlapMotor->setPowerF(flapsChannel->getValueF());
//				rightFlapMotor->setPower(aileronsChannel->getValue());
		}
		
		// Lights
		{
			BoolChannel* boolChannel;
			
			if ((boolChannel = ac.channels.getBoolChannel(ChannelType::navLights)))
				ac.lights.setNavigationEnabled(boolChannel->getValue());
			
			if ((boolChannel = ac.channels.getBoolChannel(ChannelType::strobeLights)))
				ac.lights.setStrobeEnabled(boolChannel->getValue());
			
			if ((boolChannel = ac.channels.getBoolChannel(ChannelType::landingLights)))
				ac.lights.setLandingEnabled(boolChannel->getValue());
			
			if ((boolChannel = ac.channels.getBoolChannel(ChannelType::cabinLights)))
				ac.lights.setCabinEnabled(boolChannel->getValue());
		}
	}
	
	[[noreturn]] void FlyByWire::taskBody() {
		auto& ac = Aircraft::getInstance();
		
		_computationTimeUs = esp_timer_get_time();
		
		_speedPrevMPS = ac.adirs.getAccelSpeedMPS();
		_altitudePrevM = ac.adirs.getCoordinates().getAltitude();
		_rollPrevRad = ac.adirs.getRollRad();
		_pitchPrevRad = ac.adirs.getPitchRad();
		_yawPrevRad = ac.adirs.getYawRad();
		
		while (true) {
			computeData();
			applyData();
			
			vTaskDelay(pdMS_TO_TICKS(_tickIntervalUs / 1'000));
		}
	}
}