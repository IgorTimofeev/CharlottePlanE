#pragma once

#include <cmath>
#include "settings.h"

namespace pizda {
	enum class PacketType : uint8_t {
		RemoteSetControlsValues,
		RemoteSetBooleanValues,
		RemoteSetMotorConfigurations
	};

	class Packet {
		public:
			constexpr static uint8_t headerLength = 4;
			constexpr static const char* header = "cyka";

			static void parse(uint8_t* buffer);

		private:
			static uint8_t getCRC8(const uint8_t* data, size_t length);
			static bool checkCRC(const uint8_t* data, size_t length);
	};
}