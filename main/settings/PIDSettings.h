#pragma once

#include <cstdint>

#include <NVSSettings.h>

#include "types/generic.h"

namespace pizda {
	using namespace YOBA;

	class PIDSettings : public NVSSettings {
		public:
			PIDCoefficients targetToRoll {};
			PIDCoefficients targetToPitch {};

			PIDCoefficients rollToAilerons {};
			PIDCoefficients pitchToElevator {};

		protected:
			const char* getNamespace() override {
				return _namespace;
			}

			void onRead(const NVSStream& stream) override {
				targetToRoll.p = stream.readFloat(_targetToRollP, 1);
				targetToRoll.i = stream.readFloat(_targetToRollI, 0);
				targetToRoll.d = stream.readFloat(_targetToRollD, 0);

				targetToPitch.p = stream.readFloat(_targetToPitchP, 1);
				targetToPitch.i = stream.readFloat(_targetToPitchI, 0);
				targetToPitch.d = stream.readFloat(_targetToPitchD, 0);

				rollToAilerons.p = stream.readFloat(_rollToAileronsP, 1);
				rollToAilerons.i = stream.readFloat(_rollToAileronsI, 0);
				rollToAilerons.d = stream.readFloat(_rollToAileronsD, 0);

				pitchToElevator.p = stream.readFloat(_pitchToElevatorP, 1);
				pitchToElevator.i = stream.readFloat(_pitchToElevatorI, 0);
				pitchToElevator.d = stream.readFloat(_pitchToElevatorD, 0);
			}

			void onWrite(const NVSStream& stream) override {
				stream.writeFloat(_targetToRollP, targetToRoll.p);
				stream.writeFloat(_targetToRollI, targetToRoll.i);
				stream.writeFloat(_targetToRollD, targetToRoll.d);

				stream.writeFloat(_targetToPitchP, targetToPitch.p);
				stream.writeFloat(_targetToPitchI, targetToPitch.i);
				stream.writeFloat(_targetToPitchD, targetToPitch.d);

				stream.writeFloat(_rollToAileronsP, rollToAilerons.p);
				stream.writeFloat(_rollToAileronsI, rollToAilerons.i);
				stream.writeFloat(_rollToAileronsD, rollToAilerons.d);

				stream.writeFloat(_pitchToElevatorP, pitchToElevator.p);
				stream.writeFloat(_pitchToElevatorI, pitchToElevator.i);
				stream.writeFloat(_pitchToElevatorD, pitchToElevator.d);
			}

		private:
			constexpr static auto _namespace = "fbp";

			constexpr static auto _targetToRollP = "trp";
			constexpr static auto _targetToRollI = "tri";
			constexpr static auto _targetToRollD = "trd";

			constexpr static auto _targetToPitchP = "tpp";
			constexpr static auto _targetToPitchI = "tpi";
			constexpr static auto _targetToPitchD = "tpd";

			constexpr static auto _rollToAileronsP = "rap";
			constexpr static auto _rollToAileronsI = "rai";
			constexpr static auto _rollToAileronsD = "rad";

			constexpr static auto _pitchToElevatorP = "pep";
			constexpr static auto _pitchToElevatorI = "pei";
			constexpr static auto _pitchToElevatorD = "ped";
	};
}