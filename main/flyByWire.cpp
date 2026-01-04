#include "flyByWire.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

#include <units.h>

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
	
	float FlyByWire::getSelectedSpeedMps() const {
		return _selectedSpeedMPS;
	}
	
	void FlyByWire::setSelectedSpeedMps(float selectedSpeedMps) {
		_selectedSpeedMPS = selectedSpeedMps;
	}
	
	uint16_t FlyByWire::getSelectedHeadingDeg() const {
		return _selectedHeadingDeg;
	}
	
	void FlyByWire::setSelectedHeadingDeg(uint16_t selectedHeadingDeg) {
		_selectedHeadingDeg = selectedHeadingDeg;
	}
	
	float FlyByWire::getSelectedAltitudeM() const {
		return _selectedAltitudeM;
	}
	
	void FlyByWire::setSelectedAltitudeM(float selectedAltitudeM) {
		_selectedAltitudeM = selectedAltitudeM;
	}
	
	AutopilotLateralMode FlyByWire::getLateralMode() const {
		return _lateralMode;
	}
	
	void FlyByWire::setLateralMode(AutopilotLateralMode lateralMode) {
		_lateralMode = lateralMode;
	}
	
	AutopilotVerticalMode FlyByWire::getVerticalMode() const {
		return _verticalMode;
	}
	
	void FlyByWire::setVerticalMode(AutopilotVerticalMode verticalMode) {
		_verticalMode = verticalMode;
	}
	
	bool FlyByWire::getAutothrottle() const {
		return _autothrottle;
	}
	
	void FlyByWire::setAutothrottle(bool autothrottle) {
		_autothrottle = autothrottle;
	}
	
	bool FlyByWire::getAutopilot() const {
		return _autopilot;
	}
	
	void FlyByWire::setAutopilot(bool autopilot) {
		_autopilot = autopilot;
	}
	
	float FlyByWire::getTargetRollRad() const {
		return _rollTargetRad;
	}
	
	float FlyByWire::getTargetPitchRad() const {
		return _pitchTargetRad;
	}
	
	
	float FlyByWire::getInterpolationFactor(float range, float rangeMax) {
		return std::min(std::abs(range), rangeMax) / rangeMax;
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
		
		// -------------------------------- Deltas --------------------------------
		
		const auto speedTargetAndPredictedDeltaMPS = _selectedSpeedMPS - speedPredictedMPS;
		
		const auto altitudeTargetAndPredictedDeltaM = _selectedAltitudeM - altitudePredictedM;
		
		const auto yawTargetRad = -normalizeAngleRadPi(toRadians(static_cast<float>(_selectedHeadingDeg)));
		const auto yawTargetAndPredictedDeltaRad = normalizeAngleRadPi(yawTargetRad - yawPredictedRad);
		
		// -------------------------------- Roll --------------------------------
		
		const auto yawToRight =
			(yawTargetAndPredictedDeltaRad < 0 && yawTargetAndPredictedDeltaRad < std::numbers::pi_v<float>)
			|| (yawTargetAndPredictedDeltaRad > 0 && yawTargetAndPredictedDeltaRad < -std::numbers::pi_v<float>);
		
		const bool rollToRight =
			(yawToRight && rollPredictedRad < config::flyByWire::rollAngleMaxRad)
			|| (!yawToRight && rollPredictedRad < -config::flyByWire::rollAngleMaxRad);
		
		const auto rollTargetRad =
			_lateralMode == AutopilotLateralMode::heading
			? (
				config::flyByWire::rollAngleMaxRad
				* getInterpolationFactor(
					yawTargetAndPredictedDeltaRad,
					toRadians(30)
				)
				* (rollToRight ? 1 : -1)
			)
			: 0;
		
		_rollTargetRad = LowPassFilter::applyForAngleRad(
			_rollTargetRad,
			rollTargetRad,
			LowPassFilter::getFactor(0.2f, deltaTimeUs)
		);
		
		// -------------------------------- Pitch --------------------------------
		
		float pitchTargetRad;
		
		if (_verticalMode == AutopilotVerticalMode::pitch) {
			pitchTargetRad = 0;
		}
		else {
			if (
				// Was in level change mode
				_verticalMode == AutopilotVerticalMode::levelChange
				// & become close enough to selected altitude
				&& std::abs(altitudeTargetAndPredictedDeltaM) <= Units::convertDistance(200.f, DistanceUnit::foot, DistanceUnit::meter)
			) {
				// Switching to alt hold mode
				_verticalMode = AutopilotVerticalMode::hold;
			}
			
			const auto pitchUp = altitudeTargetAndPredictedDeltaM >= 0;
			
			switch (_verticalMode) {
				case AutopilotVerticalMode::hold: {
					// Relying on altitude difference, speed doesn't matter
					pitchTargetRad =
						config::flyByWire::pitchAngleMaxRad
						* (pitchUp ? 1 : -1)
						* getInterpolationFactor(
							altitudeTargetAndPredictedDeltaM,
							Units::convertDistance(100.f, DistanceUnit::foot, DistanceUnit::meter)
						);
					
					break;
				}
				case AutopilotVerticalMode::levelChange: {
					// Relying on speed, altitude doesn't matter
					const auto speedNotEnough = speedTargetAndPredictedDeltaMPS >= 0;
					
					const auto pitchSpeedDeltaFactor = getInterpolationFactor(
						speedTargetAndPredictedDeltaMPS,
						Units::convertSpeed(2.0f, SpeedUnit::knot, SpeedUnit::meterPerSecond)
					);
					
					if (pitchUp) {
						if (speedNotEnough) {
							pitchTargetRad = 0;
						}
						else {
							pitchTargetRad =
								config::flyByWire::pitchAngleMaxRad
								* pitchSpeedDeltaFactor;
						}
					}
					else {
						if (speedNotEnough) {
							pitchTargetRad =
								-config::flyByWire::pitchAngleMaxRad
								* pitchSpeedDeltaFactor;
						}
						else {
							pitchTargetRad = 0;
						}
					};
					
					break;
				}
				// Won't ever happen
				default: {
					pitchTargetRad = 0;
					break;
				}
			}
		}
		
		_pitchTargetRad = LowPassFilter::applyForAngleRad(
			_pitchTargetRad,
			pitchTargetRad,
			LowPassFilter::getFactor(0.2f, deltaTimeUs)
		);
		
		// -------------------------------- Ailerons --------------------------------
		
		const auto rollTargetAndPredictedDeltaRad = _rollTargetRad - rollRad;
		
		const auto aileronsTargetFactor =
			_autopilot && _lateralMode != AutopilotLateralMode::roll
			? (
				0.5f
				+ (
					getInterpolationFactor(
						rollTargetAndPredictedDeltaRad,
						toRadians(20)
					)
					* config::flyByWire::aileronMaxFactor
					// [0.0; 1.0] -> [0.0; 0.5]
					/ 2.f
					* (rollTargetAndPredictedDeltaRad >= 0 ? 1 : -1)
				)
			)
			: 0.5f;
		
		_aileronsTargetFactor = LowPassFilter::applyForAngleRad(
			_aileronsTargetFactor,
			aileronsTargetFactor,
			LowPassFilter::getFactor(3.5f, deltaTimeUs)
		);
		
		// -------------------------------- Elevator --------------------------------
		
		const auto pitchTargetAndPredictedDeltaRad = _pitchTargetRad - pitchRad;
		
		const auto elevatorTargetFactor =
			_autopilot && _verticalMode != AutopilotVerticalMode::pitch
			? (
				0.5f
				+ (
					getInterpolationFactor(
						pitchTargetAndPredictedDeltaRad,
						toRadians(20)
					)
					* config::flyByWire::elevatorMaxFactor
					/ 2.f
					* (pitchTargetAndPredictedDeltaRad >= 0 ? -1 : 1)
				)
			)
			: 0.5f;
		
		_elevatorTargetFactor = LowPassFilter::applyForAngleRad(
			_elevatorTargetFactor,
			elevatorTargetFactor,
			LowPassFilter::getFactor(3.5f, deltaTimeUs)
		);
		
		// -------------------------------- Throttle --------------------------------
		
		auto throttlePitchUp =
			// Pitch control engaged
			_autothrottle
			// Pitch up
			&& altitudeTargetAndPredictedDeltaM >= Units::convertDistance(100.f, DistanceUnit::foot, DistanceUnit::meter);
		
		auto throttleState =
			// Not enough speed
			speedTargetAndPredictedDeltaMPS > 0
			// Enough, but
			|| throttlePitchUp;
		
		const auto throttleTargetFactor =
			throttlePitchUp
			? 1.0f
			: (
				0.5f
				+ interpolate(
					0.6f,
					1.0f,
					getInterpolationFactor(
						speedTargetAndPredictedDeltaMPS,
						Units::convertSpeed(10.f, SpeedUnit::knot, SpeedUnit::meterPerSecond)
					)
				)
				  / 2.f
				  * (throttleState ? 1.f : -1.f)
			);
		
		_throttleTargetFactor = LowPassFilter::applyForAngleRad(
			_throttleTargetFactor,
			throttleTargetFactor,
			LowPassFilter::getFactor(0.5f, deltaTimeUs)
		);
		
	}
	
	void FlyByWire::applyData() {
		auto& ac = Aircraft::getInstance();
		
		// Throttle
		{
			const auto motor = ac.motors.getMotor(MotorType::throttle);
			
			if (!motor)
				return;
			
			motor->setPowerF(
				_autothrottle
				? _throttleTargetFactor
				: ac.remoteData.raw.controls.throttle
			);
		}
		
		// Ailerons
		{
			const auto leftAileronMotor = ac.motors.getMotor(MotorType::leftAileron);
//				const auto rightAileronMotor = ac.motors.getMotor(MotorType::rightAileron);
			
			if (!leftAileronMotor)
				return;
			
			leftAileronMotor->setPowerF(
				_autopilot && _lateralMode != AutopilotLateralMode::roll
				? _aileronsTargetFactor
				: ac.remoteData.raw.controls.ailerons
			);
//				rightAileronMotor->setPower(aileronsChannel->getValue());
		}
		
		// Elevator & rudder
		{
			const auto leftTailMotor = ac.motors.getMotor(MotorType::leftTail);
			const auto rightTailMotor = ac.motors.getMotor(MotorType::rightTail);
			
			if (!leftTailMotor || !rightTailMotor)
				return;
			
			leftTailMotor->setPowerF(
				_autopilot && _verticalMode != AutopilotVerticalMode::pitch
				? _elevatorTargetFactor
				: ac.remoteData.raw.controls.elevator
			);
			
			rightTailMotor->setPowerF(ac.remoteData.raw.controls.rudder);
		}
		
		// Flaps
		{
			const auto leftFlapMotor = ac.motors.getMotor(MotorType::leftFlap);
//				const auto rightFlapMotor = ac.motors.getMotor(MotorType::rightAileron);
			
			if (!leftFlapMotor)
				return;
			
			leftFlapMotor->setPowerF(ac.remoteData.raw.controls.flaps);
//				rightFlapMotor->setPower(aileronsChannel->getValue());
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