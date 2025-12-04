#pragma once

#include <cmath>
#include "settings.h"
#include "YOBABitStream/main.h"

namespace pizda {
	enum class PacketType : uint8_t {
		RemoteSetAnalogValues,
		RemoteSetBooleanValues,
		RemoteSetMotorConfigurations
	};

	class Packet {
		public:
			constexpr static uint8_t headerByteCount = 2;
			constexpr static const char* header = "cy";

			constexpr static uint8_t packetTypeBitCount = 4;

			static void parse(uint8_t* buffer);

		private:
			static uint8_t getCRC8(const uint8_t* buffer, size_t length);
			static bool checkCRC(const uint8_t* buffer, size_t length);

			static bool readValueCountAndCheckCRC(
				const uint8_t* crcStart,
				ReadableBitStream& bitStream,
				uint8_t valueBitCount,
				uint8_t valueCountBitCount,
				uint8_t valueCountExpected,
				uint8_t* valueCount
			);

	};
}