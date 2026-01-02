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
	
	float FlyByWire::interpolateValueBy(float valueMin, float valueMax, float range, float rangeMax) {
		return valueMin + (valueMax - valueMin) * std::clamp<float>(std::abs(range) / rangeMax, 0, 1);
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
		const auto rollPredictedRad = _rollPrevRad + predictValue(rollPrevDeltaRad, deltaTimeUs, predictionTimeUs);
		_rollPrevRad = rollRad;
		
		// Pitch
		const auto pitchRad = ac.adirs.getPitchRad();
		const auto pitchPrevDeltaRad = pitchRad - _pitchPrevRad;
		const auto pitchPredictedRad = _pitchPrevRad + predictValue(pitchPrevDeltaRad, deltaTimeUs, predictionTimeUs);
		_pitchPrevRad = pitchRad;
		
		// Yaw
		const auto yawRad = ac.adirs.getYawRad();
		const auto yawPrevDeltaRad = yawRad - _yawPrevRad;
		const auto yawPredictedRad = _yawPrevRad + predictValue(yawPrevDeltaRad, deltaTimeUs, predictionTimeUs);
		_yawPrevRad = yawRad;
		
	
		// -------------------------------- Throttle --------------------------------
		
		const auto speedTargetMPS = ac.remoteData.raw.autopilot.speedMPS;
		const auto speedTargetAndPredictedDeltaMPS = speedTargetMPS - speedPredictedMPS;
		
		const auto altitudeTargetM = ac.remoteData.raw.autopilot.altitudeM;
		const auto altitudeTargetAndPredictedDeltaM = altitudeTargetM - altitudePredictedM;
		
		const auto throttleTargetAltitudeSafetyMarginM = 10.f;
		
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
		
		const auto throttleTargetFactor = throttleState ? 1.f : 0.f;
		
		_throttleTargetFactor = LowPassFilter::applyForAngleRad(
			_throttleTargetFactor,
			throttleTargetFactor,
			LowPassFilter::getFactor(
				interpolateValueBy(0.1, 1.0, speedTargetAndPredictedDeltaMPS, 5),
				deltaTimeUs
			)
		);
		
		// -------------------------------- Roll --------------------------------
		
		const auto yawTargetRad = -toRadians(normalizeAngle180(static_cast<float>(ac.remoteData.raw.autopilot.headingDeg)));
		const auto yawTargetAndPredictedDeltaRad = yawTargetRad - yawPredictedRad;
		
		const auto yawToRight =
			(yawTargetAndPredictedDeltaRad < 0 && yawTargetAndPredictedDeltaRad < std::numbers::pi_v<float>)
			|| (yawTargetAndPredictedDeltaRad > 0 && yawTargetAndPredictedDeltaRad < -std::numbers::pi_v<float>);
		
		const bool rollToRight =
			(yawToRight && rollPredictedRad < config::limits::autopilotRollAngleMaxRad)
			|| (!yawToRight && rollPredictedRad < -config::limits::autopilotRollAngleMaxRad);
		
		const auto rollTargetRad =
			ac.remoteData.raw.autopilot.headingHold
			? (
				interpolateValueBy(
					config::limits::autopilotRollAngleMaxRad * 0.1,
					config::limits::autopilotRollAngleMaxRad,
					yawTargetAndPredictedDeltaRad,
					toRadians(15)
				)
				* (rollToRight ? 1 : -1)
			)
			: 0;
		
		_rollTargetRad = LowPassFilter::applyForAngleRad(
			_rollTargetRad,
			rollTargetRad,
			LowPassFilter::getFactor(0.6, deltaTimeUs)
		);
		
		// -------------------------------- Pitch --------------------------------
		
		const auto pitchUp = altitudeTargetAndPredictedDeltaM >= 0;
		
		const auto pitchTargetRad =
			ac.remoteData.raw.autopilot.levelChange
			? (
				interpolateValueBy(
					config::limits::autopilotPitchAngleMaxRad * 0.1,
					config::limits::autopilotPitchAngleMaxRad,
					altitudeTargetAndPredictedDeltaM,
					100
				)
				* (pitchUp ? 1 : -1)
			)
			: 0;
		
		_pitchTargetRad = LowPassFilter::applyForAngleRad(
			_pitchTargetRad,
			pitchTargetRad,
			LowPassFilter::getFactor(0.6, deltaTimeUs)
		);
		
		// -------------------------------- Ailerons --------------------------------
		
		const auto rollTargetAndPredictedDeltaRad = _rollTargetRad - rollPredictedRad;
		
		const auto aileronsTargetFactor =
			ac.remoteData.raw.autopilot.headingHold
			? (
				0.5f
				+ (
					interpolateValueBy(0.04, 0.5, rollTargetAndPredictedDeltaRad, toRadians(20))
					* (rollTargetAndPredictedDeltaRad >= 0 ? 1 : -1)
					/ 2.f
				)
			)
			: 0.5f;
		
		_aileronsTargetFactor = LowPassFilter::applyForAngleRad(
			_aileronsTargetFactor,
			aileronsTargetFactor,
			LowPassFilter::getFactor(1.0, deltaTimeUs)
		);
		
		// -------------------------------- Elevator --------------------------------
		
		const auto pitchTargetAndPredictedDeltaRad = _pitchTargetRad - pitchPredictedRad;
		
		const auto elevatorTargetFactor =
			ac.remoteData.raw.autopilot.levelChange
			? (
				0.5f
				+ (
					interpolateValueBy(0.04, 0.5, pitchTargetAndPredictedDeltaRad, toRadians(20))
					* (pitchTargetAndPredictedDeltaRad >= 0 ? -1 : 1)
					/ 2.f
				)
			)
			: 0.5f;
		
		_elevatorTargetFactor = LowPassFilter::applyForAngleRad(
			_elevatorTargetFactor,
			elevatorTargetFactor,
			LowPassFilter::getFactor(1.0, deltaTimeUs)
		);
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