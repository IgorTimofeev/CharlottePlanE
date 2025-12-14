#pragma once

#include <cmath>
#include <functional>

#include <YOBABitStream/main.h>

#include "packet.h"

namespace pizda {
	class PacketParser {
		public:
			bool parse(uint8_t* buffer, uint8_t length);

			virtual void onConnectionLost() = 0;
			virtual void onConnectionRestored() = 0;

		protected:
			static uint8_t getCRC8(const uint8_t* buffer, size_t length);
			static bool validateChecksum(const uint8_t* buffer, size_t dataBitCount);
			static uint8_t readValueCountAndValidateChecksum(BitStream& bitStream, uint8_t valueCountBitCount, uint8_t valueBitCount);

			virtual bool onParse(BitStream& stream, PacketType packetType) = 0;

		private:
			uint8_t parseOne(uint8_t* buffer);
	};
}