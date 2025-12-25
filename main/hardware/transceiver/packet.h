#pragma once

#include <cmath>

#include "settings/settings.h"

namespace pizda {
	enum class PacketType : uint8_t {
		remoteChannelDataStructure,
		remoteChannelData,
		remoteMotorConfiguration,
		remoteBaro,

		aircraftADIRS
	};

	class Packet {
		public:
			constexpr static uint8_t headerLengthBytes = 2;
			constexpr static uint8_t header[] { 'c', 'y' };

			constexpr static uint8_t typeLengthBits = 4;
			constexpr static uint8_t checksumLengthBytes = 1;
			
		private:

	};
}