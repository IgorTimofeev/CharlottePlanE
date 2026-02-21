#pragma once

#include <cstdint>

#include <NVSSettings.h>

#include "types/generic.h"
#include "utilities/math.h"

namespace pizda {
	using namespace YOBA;

	class AutopilotSettingsPIDs {
		public:
			PIDCoefficients yawToRoll {};
			PIDCoefficients altitudeToPitch {};
			PIDCoefficients speedToPitch {};

			PIDCoefficients rollToAilerons {};
			PIDCoefficients pitchToElevator {};

			PIDCoefficients speedToThrottle {};

			static void read(const NVSStream& stream, const char* keyP, const char* keyI, const char* keyD, PIDCoefficients& coefficients) {
				coefficients.p = stream.readFloat(keyP, 1);
				coefficients.i = stream.readFloat(keyI, 0);
				coefficients.d = stream.readFloat(keyD, 0);
			}

			static void write(const NVSStream& stream, const char* keyP, const char* keyI, const char* keyD, const PIDCoefficients& coefficients) {
				stream.writeFloat(keyP, coefficients.p);
				stream.writeFloat(keyI, coefficients.i);
				stream.writeFloat(keyD, coefficients.d);
			}
	};

	class AutopilotSettings : public NVSSettings {
		public:
			// Lateral
			float maxRollAngleRad = 0;
			float stabilizedModeRollAngleIncrementFactorPerSecond = 0;
			float rollAngleLPFFactorPerSecond = 0;
			float maxAileronsFactor = 0;

			// Vertical
			float maxPitchAngleRad = 0;
			float stabilizedModePitchAngleIncrementFactorPerSecond = 0;
			float pitchAngleLPFFactorPerSecond = 0;
			float maxElevatorFactor = 0;

			// Longitudinal
			float throttleLPFFactorPerSecond = 0;

			// PIDs
			AutopilotSettingsPIDs PIDs {};

		protected:
			const char* getNamespace() override {
				return _namespace;
			}

			void onRead(const NVSStream& stream) override {
				// Lateral
				maxRollAngleRad = stream.readFloat(_maxRollAngleRad, toRadians(30));
				stabilizedModeRollAngleIncrementFactorPerSecond = stream.readFloat(_stabilizedModeRollAngleIncrementFactorPerSecond, 30.f);
				rollAngleLPFFactorPerSecond = stream.readFloat(_rollAngleLPFFactorPerSecond, 0.5f);
				maxAileronsFactor = stream.readFloat(_maxAileronsFactor, 1.0f);

				// Vertical
				maxPitchAngleRad = stream.readFloat(_maxPitchAngleRad, toRadians(20));
				stabilizedModePitchAngleIncrementFactorPerSecond = stream.readFloat(_stabilizedModePitchAngleIncrementFactorPerSecond, 20.f);
				pitchAngleLPFFactorPerSecond = stream.readFloat(_pitchAngleLPFFactorPerSecond, 0.5f);
				maxElevatorFactor = stream.readFloat(_maxElevatorFactor, 1.0f);

				// Longitudinal
				throttleLPFFactorPerSecond = stream.readFloat(_throttleLPFFactorPerSecond, 0.5f);

				// PIDs
				AutopilotSettingsPIDs::read(stream, _yawToRollP, _yawToRollI, _yawToRollD, PIDs.yawToRoll);
				AutopilotSettingsPIDs::read(stream, _altitudeToPitchP, _altitudeToPitchI, _altitudeToPitchD, PIDs.altitudeToPitch);
				AutopilotSettingsPIDs::read(stream, _speedToPitchP, _speedToPitchI, _speedToPitchD, PIDs.speedToPitch);
				AutopilotSettingsPIDs::read(stream, _rollToAileronsP, _rollToAileronsI, _rollToAileronsD, PIDs.rollToAilerons);
				AutopilotSettingsPIDs::read(stream, _pitchToElevatorP, _pitchToElevatorI, _pitchToElevatorD, PIDs.pitchToElevator);
				AutopilotSettingsPIDs::read(stream, _speedToThrottleP, _speedToThrottleI, _speedToThrottleD, PIDs.speedToThrottle);
			}

			void onWrite(const NVSStream& stream) override {
				// Lateral
				stream.writeFloat(_maxRollAngleRad, maxRollAngleRad);
				stream.writeFloat(_stabilizedModeRollAngleIncrementFactorPerSecond, stabilizedModeRollAngleIncrementFactorPerSecond);
				stream.writeFloat(_rollAngleLPFFactorPerSecond, rollAngleLPFFactorPerSecond);
				stream.writeFloat(_maxAileronsFactor, maxAileronsFactor);

				// Vertical
				stream.writeFloat(_maxPitchAngleRad, maxPitchAngleRad);
				stream.writeFloat(_stabilizedModePitchAngleIncrementFactorPerSecond, stabilizedModePitchAngleIncrementFactorPerSecond);
				stream.writeFloat(_pitchAngleLPFFactorPerSecond, pitchAngleLPFFactorPerSecond);
				stream.writeFloat(_maxElevatorFactor, maxElevatorFactor);

				// Longitudinal
				stream.writeFloat(_throttleLPFFactorPerSecond, throttleLPFFactorPerSecond);

				// PIDs
				AutopilotSettingsPIDs::write(stream, _yawToRollP, _yawToRollI, _yawToRollD, PIDs.yawToRoll);
				AutopilotSettingsPIDs::write(stream, _altitudeToPitchP, _altitudeToPitchI, _altitudeToPitchD, PIDs.altitudeToPitch);
				AutopilotSettingsPIDs::write(stream, _speedToPitchP, _speedToPitchI, _speedToPitchD, PIDs.speedToPitch);
				AutopilotSettingsPIDs::write(stream, _rollToAileronsP, _rollToAileronsI, _rollToAileronsD, PIDs.rollToAilerons);
				AutopilotSettingsPIDs::write(stream, _pitchToElevatorP, _pitchToElevatorI, _pitchToElevatorD, PIDs.pitchToElevator);
				AutopilotSettingsPIDs::write(stream, _speedToThrottleP, _speedToThrottleI, _speedToThrottleD, PIDs.speedToThrottle);
			}

		private:
			constexpr static auto _namespace = "ap1";

			// Lateral
			constexpr static auto _maxRollAngleRad = "mrla";
			constexpr static auto _stabilizedModeRollAngleIncrementFactorPerSecond = "raif";
			constexpr static auto _rollAngleLPFFactorPerSecond = "ralf";
			constexpr static auto _maxAileronsFactor = "aiff";

			// Vertical
			constexpr static auto _maxPitchAngleRad = "mpia";
			constexpr static auto _stabilizedModePitchAngleIncrementFactorPerSecond = "paif";
			constexpr static auto _pitchAngleLPFFactorPerSecond = "palf";
			constexpr static auto _maxElevatorFactor = "elff";

			// Longitudinal
			constexpr static auto _throttleLPFFactorPerSecond = "tlpf";

			// PIDs
			constexpr static auto _yawToRollP = "pyrp";
			constexpr static auto _yawToRollI = "pyri";
			constexpr static auto _yawToRollD = "pyrd";

			constexpr static auto _altitudeToPitchP = "papp";
			constexpr static auto _altitudeToPitchI = "papi";
			constexpr static auto _altitudeToPitchD = "papd";

			constexpr static auto _speedToPitchP = "pspp";
			constexpr static auto _speedToPitchI = "pspi";
			constexpr static auto _speedToPitchD = "pspd";

			constexpr static auto _rollToAileronsP = "prap";
			constexpr static auto _rollToAileronsI = "prai";
			constexpr static auto _rollToAileronsD = "prad";

			constexpr static auto _pitchToElevatorP = "ppep";
			constexpr static auto _pitchToElevatorI = "ppei";
			constexpr static auto _pitchToElevatorD = "pped";

			constexpr static auto _speedToThrottleP = "pstp";
			constexpr static auto _speedToThrottleI = "psti";
			constexpr static auto _speedToThrottleD = "pstd";
	};
}
