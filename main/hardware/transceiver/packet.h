#pragma once

#include <cmath>

#include "settings/settings.h"

namespace pizda {
	enum class PacketType : uint8_t {
		RemoteChannelDataStructure,
		RemoteChannelData,
		RemoteMotorConfiguration,

		AircraftAHRS
	};

	class Packet {
		public:
			constexpr static uint8_t headerLengthBytes = 2;
			constexpr static const char* header = "cy";

			constexpr static uint8_t typeLengthBits = 4;

		private:

	};
}