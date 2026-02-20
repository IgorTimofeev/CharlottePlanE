#include "systems/flyByWire/flyByWire.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

#include <units.h>
#include <lowPassFilter.h>

#include "config.h"
#include "aircraft.h"
#include "utilities/math.h"

namespace pizda {
	void FlyByWire::setup() {
		xTaskCreate(
			[](void* arg) {
				static_cast<FlyByWire*>(arg)->onStart();
			},
			"FlyByWire",
			4 * 1024,
			this,
			20,
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
	
	float FlyByWire::predictValue(const float valueDelta, const float deltaTimeS, const float dueTimeS) {
		// valueDelta - deltaTimeS
		// x          - dueTimeS
		return valueDelta * dueTimeS / deltaTimeS;
	}
	
	void FlyByWire::computeData() {
		auto& ac = Aircraft::getInstance();
		
		const auto deltaTimeS = static_cast<float>(esp_timer_get_time() - _computationTimeUs) / 1'000'000;
		_computationTimeUs = esp_timer_get_time();
		
		// -------------------------------- Prediction --------------------------------
		
		// Speed
		const auto speedMPS = ac.adirs.getAirspeedMPS();
		const auto speedPrevDeltaMPS = speedMPS - _speedPrevMPS;
		const auto speedPredictedMPS = _speedPrevMPS + predictValue(speedPrevDeltaMPS, deltaTimeS, 2);
		_speedPrevMPS = speedMPS;
		
		// Altitude
		const auto altitudeM = ac.adirs.getCoordinates().getAltitude();
		const auto altitudePrevDeltaM = altitudeM - _altitudePrevM;
		const auto altitudePredictedM = _altitudePrevM + predictValue(altitudePrevDeltaM, deltaTimeS, 1);
		_altitudePrevM = altitudeM;
		
		// Roll
		const auto rollRad = ac.adirs.getRollRad();
		const auto rollPrevDeltaRad = rollRad - _rollPrevRad;
		const auto rollPredictedRad = _rollPrevRad + predictValue(rollPrevDeltaRad, deltaTimeS, 1);
		_rollPrevRad = rollRad;
		
		// Pitch
		const auto pitchRad = ac.adirs.getPitchRad();
		// const auto pitchPrevDeltaRad = pitchRad - _pitchPrevRad;
		// const auto pitchPredictedRad = _pitchPrevRad + predictValue(pitchPrevDeltaRad, deltaTimeS, 1);
		// _pitchPrevRad = pitchRad;
		
		// Yaw
		const auto yawRad = ac.adirs.getYawRad();
		const auto yawPrevDeltaRad = yawRad - _yawPrevRad;
		const auto yawPredictedRad = _yawPrevRad + predictValue(yawPrevDeltaRad, deltaTimeS, 1);
		_yawPrevRad = yawRad;

		// -------------------------------- Deltas --------------------------------

		const auto speedTargetAndPredictedDeltaMPS = _speedSelectedMPS - speedPredictedMPS;

		const auto altitudeTargetAndPredictedDeltaM = _altitudeSelectedM - altitudePredictedM;


		// -------------------------------- Lateral --------------------------------

		float rollTargetRad = 0;

		if (_lateralMode == AutopilotLateralMode::dir) {
			rollTargetRad = 0;
		}
		else if (_lateralMode == AutopilotLateralMode::stab) {
			if (_autopilot) {
				rollTargetRad = std::clamp(
					_rollTargetRad + (ac.remoteData.raw.controls.ailerons * 2 - 1) * config::flyByWire::rollAngleMaxRad,
					-config::flyByWire::rollAngleMaxRad,
					config::flyByWire::rollAngleMaxRad
				);
			}
			else {
				rollTargetRad = 0;
			}
		}
		else {
			const auto yawTargetRad = -normalizeAngleRadPi(toRadians(static_cast<float>(_headingSelectedDeg)));
			const auto yawTargetDeltaRad = normalizeAngleRadPi(yawTargetRad - yawRad);

			rollTargetRad = _targetToRollPID.tick(
				yawTargetDeltaRad,
				0,

				ac.settings.PID.targetToRoll.p,
				ac.settings.PID.targetToRoll.i,
				ac.settings.PID.targetToRoll.d,

				deltaTimeS,

				-config::flyByWire::rollAngleMaxRad,
				config::flyByWire::rollAngleMaxRad
			);
		}

		_rollTargetRad = LowPassFilter::applyToAngle(
			_rollTargetRad,
			rollTargetRad,
			LowPassFilter::getDeltaTimeSFactor(0.7f, deltaTimeS)
		);

		// -------------------------------- Vertical --------------------------------

		float pitchTargetRad = 0;

		if (_verticalMode == AutopilotVerticalMode::dir) {
			pitchTargetRad = 0;
		}
		else if (_verticalMode == AutopilotVerticalMode::stab) {
			if (_autopilot) {
				pitchTargetRad = std::clamp(
					_pitchTargetRad - (ac.remoteData.raw.controls.elevator * 2 - 1) * config::flyByWire::pitchAngleMaxRad,
					-config::flyByWire::pitchAngleMaxRad,
					config::flyByWire::pitchAngleMaxRad
				);
			}
			else {
				pitchTargetRad = 0;
			}
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

		_pitchTargetRad = LowPassFilter::applyToAngle(
			_pitchTargetRad,
			pitchTargetRad,
			LowPassFilter::getDeltaTimeSFactor(0.7f, deltaTimeS)
		);

		// -------------------------------- Ailerons --------------------------------

		if (_autopilot && _lateralMode != AutopilotLateralMode::dir) {
			const auto rollTargetDeltaRad = normalizeAngleRadPi(_rollTargetRad - rollRad);

			_aileronsFactor = _rollToAileronsPID.tick(
				rollTargetDeltaRad,
				0,

				ac.settings.PID.rollToAilerons.p,
				ac.settings.PID.rollToAilerons.i,
				ac.settings.PID.rollToAilerons.d,

				deltaTimeS,

				-1.f,
				1.f
			);

			_aileronsFactor = 1.f - (_aileronsFactor + 1.f) / 2.f;
		}
		else {
			_aileronsFactor = std::clamp(ac.remoteData.raw.controls.ailerons + ac.settings.trim.aileronsTrim, 0.f, 1.f);
		}

		// -------------------------------- Elevator --------------------------------

		if (_autopilot && _verticalMode != AutopilotVerticalMode::dir) {
			const auto pitchTargetDeltaRad = normalizeAngleRadPi(_pitchTargetRad - pitchRad);

			_elevatorFactor = _pitchToElevatorPID.tick(
				pitchTargetDeltaRad,
				0,

				ac.settings.PID.pitchToElevator.p,
				ac.settings.PID.pitchToElevator.i,
				ac.settings.PID.pitchToElevator.d,

				deltaTimeS,

				-1.f,
				1.f
			);

			_elevatorFactor = (_elevatorFactor + 1.f) / 2.f;
		}
		else {
			_elevatorFactor = std::clamp(ac.remoteData.raw.controls.elevator + ac.settings.trim.elevatorTrim, 0.f, 1.f);
		}

		// -------------------------------- Rudder --------------------------------

		_rudderFactor = std::clamp(ac.remoteData.raw.controls.rudder + ac.settings.trim.rudderTrim, 0.f, 1.f);

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
		
		_throttleTargetFactor = LowPassFilter::apply(
			_throttleTargetFactor,
			throttleTargetFactor,
			LowPassFilter::getDeltaTimeSFactor(throttleLPFFactor, deltaTimeS)
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

			leftAileronMotor->setPowerF(_aileronsFactor);
			rightAileronMotor->setPowerF(_aileronsFactor);
		}
		
		// Elevator & rudder
		{
			const auto leftTailMotor = ac.motors.getMotor(MotorType::tailLeft);
			const auto rightTailMotor = ac.motors.getMotor(MotorType::tailRight);
			const auto noseWheelMotor = ac.motors.getMotor(MotorType::noseWheel);

			#ifdef SIM
				leftTailMotor->setPowerF(_elevatorFactor);
				rightTailMotor->setPowerF(_rudderFactor);

			#else
				// V-tail mixing
				const auto elevatorPowerShifted = _elevatorFactor * 2 - 1;
				const auto rudderPowerShifted = _rudderFactor * 2 - 1;
				const auto leftPower = (std::clamp(elevatorPowerShifted + rudderPowerShifted, -1.f, 1.f) + 1.f) / 2.f;
				const auto rightPower = (std::clamp(elevatorPowerShifted - rudderPowerShifted, -1.f, 1.f) + 1.f) / 2.f;

				// ESP_LOGI(_logTag, "elevatorPower: %f, rudderPower: %f, leftPower: %f, rightPower: %f", _elevatorTargetFactor, _rudderFactor, leftPower, rightPower);

				leftTailMotor->setPowerF(leftPower);
				rightTailMotor->setPowerF(rightPower);
			#endif

			// Nose wheel
			noseWheelMotor->setPowerF(ac.remoteData.raw.controls.rudder);
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
	
	[[noreturn]] void FlyByWire::onStart() {
		const auto& ac = Aircraft::getInstance();
		
		_computationTimeUs = esp_timer_get_time();
		
		_speedPrevMPS = ac.adirs.getAirspeedMPS();
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