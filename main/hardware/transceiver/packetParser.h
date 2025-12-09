#pragma once

#include <cmath>
#include <functional>

#include <YOBABitStream/main.h>

#include "packet.h"

namespace pizda {
	class PacketParser {
		public:
			uint8_t parseOne(uint8_t* buffer);

			void parse(uint8_t* buffer, uint8_t length);

		protected:
			virtual bool onParse(ReadableBitStream& stream, PacketType packetType) = 0;

			static uint8_t getCRC8(const uint8_t* buffer, size_t length);
			static bool validateChecksum(const uint8_t* buffer, size_t dataBitCount);
			static uint8_t readValueCountAndValidateChecksum(ReadableBitStream& bitStream, uint8_t valueCountBitCount, uint8_t valueBitCount);
	};
}