#pragma once

#include <cmath>
#include <functional>

#include <YOBABitStream/main.h>

#include "packet.h"
#include "packetParser.h"

namespace pizda {
	class Transceiver {
		public:
			void setup();
			void setPacketParser(PacketParser* packetParser);
			void start();

		private:
			PacketParser* packetParser = nullptr;

			constexpr static uint16_t readingBufferLength = 255;
			uint8_t readingBuffer[readingBufferLength] {};

			static void readingTask(void *arg);
	};;
}