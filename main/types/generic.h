#pragma once

#include <cstdint>
#include <algorithm>

#include "config.h"

namespace pizda {
	enum class ConnectionState : uint8_t {
		initial,
		connected,
		disconnected
	};
	
	enum class AircraftCalibrationSystem : uint8_t {
		accelAndGyro,
		mag
	};
	
	enum class AutopilotLateralMode : uint8_t {
		man,
		hdg
	};
	
	enum class AutopilotVerticalMode : uint8_t {
		man,
		flc,
		alts,
		alt
	};
	
	class MotorSettings {
		public:
			uint16_t min = 1000;
			uint16_t max = 2000;
			bool reverse = false;

			void sanitize() {
				min = std::clamp<uint16_t>(min, 0, 3000);
				max = std::clamp<uint16_t>(max, 0, 3000);

				if (min > max)
					std::swap(min, max);
			}
	};

	class TransceiverCommunicationSettings {
		public:
			uint32_t RFFrequencyHz = config::transceiver::RFFrequencyHz;
			SX1262::LoRaBandwidth bandwidth = config::transceiver::bandwidth;
			uint8_t spreadingFactor = config::transceiver::spreadingFactor;
			SX1262::LoRaCodingRate codingRate = config::transceiver::codingRate;
			uint8_t syncWord = config::transceiver::syncWord;
			int8_t powerDBm = config::transceiver::powerDBm;
			uint16_t preambleLength = config::transceiver::preambleLength;

			void sanitize() {
				RFFrequencyHz = std::clamp<uint32_t>(RFFrequencyHz, 120'000'000, 960'000'000);
				spreadingFactor = std::clamp<uint8_t>(spreadingFactor, 5, 12);
				powerDBm = std::clamp<int8_t>(powerDBm, -17, 22);
			}
	};

	class PIDCoefficients {
		public:
			float p, i, d;
	};
}
