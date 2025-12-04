#pragma once

#include <cmath>
#include "settings.h"
#include "YOBABitStream/main.h"

namespace pizda {
	enum class PacketType : uint8_t {
		RemoteSetMotorValues,
		RemoteSetMotorConfigurations,
		RemoteSetBooleanValues,
	};

	class Packet {
		public:
			constexpr static uint8_t headerLength = 4;
			constexpr static const char* header = "cyka";

			static void parse(uint8_t* buffer);

		private:
			static uint8_t getCRC8(const uint8_t* data, size_t length);
			static bool checkCRC(const uint8_t* data, size_t length);

			static bool computeByteCountAndCheckCRC(uint8_t* buffer, uint32_t bitCount);
			static bool readValueCountAndCheckCRC(
				uint8_t* buffer,
				ReadableBitStream& bitStream,
				uint8_t valueBitCount,
				uint8_t valueCountBitCount,
				uint8_t valueCountExpected,
				uint8_t* valueCount
			);
	};
}