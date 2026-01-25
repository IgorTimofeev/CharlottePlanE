#include "systems/flyByWire.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

#include <units.h>

#include "config.h"
#include "aircraft.h"
#include "utilities/math.h"
#include "utilities/lowPassFilter.h"

namespace pizda {
	void FlyByWire::setup() {
		xTaskCreate(
			[](void* arg) {
				static_cast<FlyByWire*>(arg)->taskBody();
			},
			"Autopilot",
			4 * 1024,
			this,
			16,
			nullptr
		);
	}
	
	float FlyByWire::getSelectedSpeedMps() const {
		return _speedSelectedMPS;
	}
	
	void FlyByWire::setSelectedSpeedMps(const float value) {
		_speedSelectedMPS = value;
	}
	
	uint16_t FlyByWire::getSelectedHeadingDeg() const {
		return _headingSelectedDeg;
	}
	
	void FlyByWire::setSelectedHeadingDeg(const uint16_t value) {
		_headingSelectedDeg = value;
	}
	
	float FlyByWire::getSelectedAltitudeM() const {
		return _altitudeSelectedM;
	}
	
	void FlyByWire::setSelectedAltitudeM(const float value) {
		_altitudeSelectedM = value;
	}
	
	AutopilotLateralMode FlyByWire::getLateralMode() const {
		return _lateralMode;
	}
	
	void FlyByWire::setLateralMode(const AutopilotLateralMode value) {
		_lateralMode = value;
	}
	
	AutopilotVerticalMode FlyByWire::getVerticalMode() const {
		return _verticalMode;
	}
	
	void FlyByWire::setVerticalMode(const AutopilotVerticalMode value) {
		_verticalMode = value;
	}
	
	bool FlyByWire::getAutothrottle() const {
		return _autothrottle;
	}
	
	void FlyByWire::setAutothrottle(const bool value) {
		_autothrottle = value;
	}
	
	bool FlyByWire::getAutopilot() const {
		return _autopilot;
	}
	
	void FlyByWire::setAutopilot(const bool value) {
		_autopilot = value;
	}
	
	float FlyByWire::getTargetRollRad() const {
		return _rollTargetRad;
	}
	
	float FlyByWire::getTargetPitchRad() const {
		return _pitchTargetRad;
	}
	
	float FlyByWire::mapPizda(const float min, const float max, const float factor) {
		return min + (max - min) * factor;
	}
	
	float FlyByWire::getInterpolationFactor(const float range, const float rangeMax) {
		return std::min(std::abs(range), rangeMax) / rangeMax;
	}
	
	float FlyByWire::predictValue(const float valueDelta, const uint32_t deltaTimeUs, const uint32_t dueTimeUs) {
		// valueDelta - deltaTimeUs
		// x          - dueTimeUs
		return valueDelta * static_cast<float>(dueTimeUs) / static_cast<float>(deltaTimeUs);
	}
	
	void FlyByWire::computeData() {
		auto& ac = Aircraft::getInstance();
		
		const auto deltaTimeUs = esp_timer_get_time() - _computationTimeUs;
		_computationTimeUs = esp_timer_get_time();
		
		// -------------------------------- Prediction --------------------------------
		
		// Speed
		const auto speedMPS = ac.adirs.getAccelSpeedMPS();
		const auto speedPrevDeltaMPS = speedMPS - _speedPrevMPS;
		const auto speedPredictedMPS = _speedPrevMPS + predictValue(speedPrevDeltaMPS, deltaTimeUs, 2'000'000);
		_speedPrevMPS = speedMPS;
		
		// Altitude
		const auto altitudeM = ac.adirs.getCoordinates().getAltitude();
		const auto altitudePrevDeltaM = altitudeM - _altitudePrevM;
		const auto altitudePredictedM = _altitudePrevM + predictValue(altitudePrevDeltaM, deltaTimeUs, 1'000'000);
		_altitudePrevM = altitudeM;
		
		// Roll
		const auto rollRad = ac.adirs.getRollRad();
		const auto rollPrevDeltaRad = rollRad - _rollPrevRad;
		const auto rollPredictedRad = _rollPrevRad + predictValue(rollPrevDeltaRad, deltaTimeUs, 1'000'000);
		_rollPrevRad = rollRad;
		
		// Pitch
		const auto pitchRad = ac.adirs.getPitchRad();
		const auto pitchPrevDeltaRad = pitchRad - _pitchPrevRad;
		const auto pitchPredictedRad = _pitchPrevRad + predictValue(pitchPrevDeltaRad, deltaTimeUs, 1'000'000);
		_pitchPrevRad = pitchRad;
		
		// Yaw
		const auto yawRad = ac.adirs.getYawRad();
		const auto yawPrevDeltaRad = yawRad - _yawPrevRad;
		const auto yawPredictedRad = _yawPrevRad + predictValue(yawPrevDeltaRad, deltaTimeUs, 1'000'000);
		_yawPrevRad = yawRad;
		
		// -------------------------------- Deltas --------------------------------
		
		const auto speedTargetAndPredictedDeltaMPS = _speedSelectedMPS - speedPredictedMPS;
		
		const auto altitudeTargetAndPredictedDeltaM = _altitudeSelectedM - altitudePredictedM;
		
		const auto yawTargetRad = -normalizeAngleRadPi(toRadians(static_cast<float>(_headingSelectedDeg)));
		const auto yawTargetAndPredictedDeltaRad = normalizeAngleRadPi(yawTargetRad - yawPredictedRad);
		
		// -------------------------------- Roll --------------------------------
		
		const auto yawToRight =
			(yawTargetAndPredictedDeltaRad < 0 && yawTargetAndPredictedDeltaRad < std::numbers::pi_v<float>)
			|| (yawTargetAndPredictedDeltaRad > 0 && yawTargetAndPredictedDeltaRad < -std::numbers::pi_v<float>);
		
		const bool rollToRight =
			(yawToRight && rollPredictedRad < config::flyByWire::rollAngleMaxRad)
			|| (!yawToRight && rollPredictedRad < -config::flyByWire::rollAngleMaxRad);
		
		const auto rollTargetRad =
			_lateralMode == AutopilotLateralMode::hdg
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
			LowPassFilter::getFactor(0.5f, deltaTimeUs)
		);
		
		// -------------------------------- Pitch --------------------------------
		
		float pitchTargetRad;
		
		if (_verticalMode == AutopilotVerticalMode::man) {
			pitchTargetRad = 0;
		}
		else {
			if (
				// Was in level change mode
				_verticalMode == AutopilotVerticalMode::flc
				// & become close enough to selected altitude
				&& std::abs(altitudeTargetAndPredictedDeltaM) <= config::flyByWire::altitudeDeltaForFLCToALTSSwitchM
			) {
				// Switching to altitude selected mode
				_verticalMode = AutopilotVerticalMode::alts;
			}
			
			const auto pitchUp = altitudeTargetAndPredictedDeltaM >= 0;
			
			switch (_verticalMode) {
				case AutopilotVerticalMode::alts:
				case AutopilotVerticalMode::alt: {
					// Relying on altitude difference, speed doesn't matter
					pitchTargetRad =
						config::flyByWire::pitchAngleMaxRad
						* (pitchUp ? 1 : -1)
						* getInterpolationFactor(
							altitudeTargetAndPredictedDeltaM,
							config::flyByWire::altitudeDeltaForFLCToALTSSwitchM
						);
					
					break;
				}
				case AutopilotVerticalMode::flc: {
					// Relying on speed, altitude doesn't matter
					const auto speedLow = speedTargetAndPredictedDeltaMPS >= 0;
					
					const auto pitchSpeedDeltaFactor = getInterpolationFactor(
						speedTargetAndPredictedDeltaMPS,
						Units::convertSpeed(2.0f, SpeedUnit::knot, SpeedUnit::meterPerSecond)
					);
					
					if (pitchUp) {
						if (speedLow) {
							pitchTargetRad = 0;
						}
						else {
							pitchTargetRad =
								config::flyByWire::pitchAngleMaxRad
								* pitchSpeedDeltaFactor;
						}
					}
					else {
						if (speedLow) {
							pitchTargetRad =
								-config::flyByWire::pitchAngleMaxRad
								* pitchSpeedDeltaFactor;
						}
						else {
							pitchTargetRad = 0;
						}
					}

					break;
				}
				// Won't ever happen, BUT
				default: {
					pitchTargetRad = 0;
					break;
				}
			}
		}
		
		_pitchTargetRad = LowPassFilter::applyForAngleRad(
			_pitchTargetRad,
			pitchTargetRad,
			LowPassFilter::getFactor(0.5f, deltaTimeUs)
		);
		
		// -------------------------------- Ailerons --------------------------------
		
		const auto rollTargetAndPredictedDeltaRad = _rollTargetRad - rollRad;
		
		const auto aileronsTargetFactor =
			_autopilot && _lateralMode != AutopilotLateralMode::man
			? (
				0.5f
				+ (
					getInterpolationFactor(
						rollTargetAndPredictedDeltaRad,
						toRadians(12)
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
			LowPassFilter::getFactor(4.5f, deltaTimeUs)
		);
		
		// -------------------------------- Elevator --------------------------------
		
		const auto pitchTargetAndPredictedDeltaRad = _pitchTargetRad - pitchRad;
		
		const auto elevatorTargetFactor =
			_autopilot && _verticalMode != AutopilotVerticalMode::man
			? (
				0.5f
				+ (
					getInterpolationFactor(
						pitchTargetAndPredictedDeltaRad,
						toRadians(12)
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
			LowPassFilter::getFactor(4.5f, deltaTimeUs)
		);
		
		// -------------------------------- Throttle --------------------------------
		
		float throttleTargetFactor;
		float throttleLPFFactor;
		
		if (_autothrottle) {
			const auto climb = altitudeTargetAndPredictedDeltaM > 0;
			const auto speedLow = speedTargetAndPredictedDeltaMPS > 0;
			
			// FLC & climb
			if (_verticalMode == AutopilotVerticalMode::flc) {
				throttleTargetFactor = climb ? 1.f : 0.f;
				throttleLPFFactor = 0.5f;
			}
			// Man, hold
			else {
				throttleTargetFactor = speedLow ? 1.f : 0.f;
				
				throttleLPFFactor = mapPizda(
					0.01f,
					0.8f,
					getInterpolationFactor(
						speedTargetAndPredictedDeltaMPS,
						Units::convertSpeed(8.f, SpeedUnit::knot, SpeedUnit::meterPerSecond)
					)
				);
			}
		}
		else {
			throttleTargetFactor = 0;
			throttleLPFFactor = 0.5f;
		}
		
		_throttleTargetFactor = LowPassFilter::applyForAngleRad(
			_throttleTargetFactor,
			throttleTargetFactor,
			LowPassFilter::getFactor(throttleLPFFactor, deltaTimeUs)
		);
	}
	
	void FlyByWire::applyData() const {
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
			const auto leftAileronMotor = ac.motors.getMotor(MotorType::aileronLeft);
			const auto rightAileronMotor = ac.motors.getMotor(MotorType::aileronRight);
			
			if (!leftAileronMotor || !rightAileronMotor)
				return;
			
			const auto power = std::clamp(
				// Trim
				ac.settings.trim.aileronsTrim
				// Value
				+ (
					_autopilot && _lateralMode != AutopilotLateralMode::man
					? _aileronsTargetFactor
					: ac.remoteData.raw.controls.ailerons
				),
				0.f,
				1.f
			);
				
			leftAileronMotor->setPowerF(power);
			rightAileronMotor->setPower(power);
		}
		
		// Elevator & rudder
		{
			const auto leftTailMotor = ac.motors.getMotor(MotorType::tailLeft);
			const auto rightTailMotor = ac.motors.getMotor(MotorType::tailRight);
			
			if (!leftTailMotor || !rightTailMotor)
				return;
			
			const auto elevatorPower = std::clamp(
				// Trim
				ac.settings.trim.elevatorTrim
				// Value
				+ (
					_autopilot && _verticalMode != AutopilotVerticalMode::man
					? _elevatorTargetFactor
					: ac.remoteData.raw.controls.elevator
				),
				0.f,
				1.f
			);
			
			const auto rudderPower = std::clamp(
				ac.settings.trim.rudderTrim
				+ (
					ac.remoteData.raw.controls.rudder
				),
				0.f,
				1.f
			);
			
			// V-tail mixing
			// ...
			
			leftTailMotor->setPowerF(elevatorPower);
			rightTailMotor->setPowerF(rudderPower);
		}
		
		// Flaps
		{
			const auto leftFlapMotor = ac.motors.getMotor(MotorType::flapLeft);
			const auto rightFlapMotor = ac.motors.getMotor(MotorType::flapRight);
			
			if (!leftFlapMotor || !rightFlapMotor)
				return;
			
			leftFlapMotor->setPowerF(ac.remoteData.raw.controls.flaps);
			rightFlapMotor->setPowerF(ac.remoteData.raw.controls.flaps);
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
			
			vTaskDelay(pdMS_TO_TICKS(1'000 / _tickFrequencyHz));
		}
	}
}