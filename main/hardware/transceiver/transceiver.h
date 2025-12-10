#pragma once

#include <cmath>
#include <functional>

#include <YOBABitStream/main.h>

#include "packet.h"
#include "packetParser.h"

namespace pizda {
	enum class TransceiverConnectionState : uint8_t {
		initial,
		normal,
		lost
	};

	class Transceiver {
		public:
			void setup();
			void setPacketParser(PacketParser* packetParser);
			void start();

		private:
			constexpr static uint32_t _connectionLostInterval = 3'000'000;

			PacketParser* _packetParser = nullptr;
			uint32_t _connectionLostTime = 0;
			TransceiverConnectionState _connectionState = TransceiverConnectionState::initial;

			constexpr static uint16_t _readingBufferLength = 255;
			uint8_t _readingBuffer[_readingBufferLength] {};

			static void readingTask(void *arg);
			void onReadingTaskTick();
			void updateConnectionLostTime();
	};;
}