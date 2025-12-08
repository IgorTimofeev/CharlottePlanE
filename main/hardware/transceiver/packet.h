#pragma once

#include <cmath>
#include "settings.h"
#include "YOBABitStream/main.h"

namespace pizda {
	enum class PacketType : uint8_t {
		RemoteChannelDataStructure,
		RemoteChannelData,
		RemoteMotorConfiguration,

		AircraftAHRS
	};

	class Packet {
		public:
			constexpr static uint8_t headerByteCount = 2;
			constexpr static const char* header = "cy";

			constexpr static uint8_t typeBitCount = 4;

		private:

	};
}