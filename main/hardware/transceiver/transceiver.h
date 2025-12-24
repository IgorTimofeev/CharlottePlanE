#pragma once

#include <cmath>
#include <functional>

#include <bitStream.h>
#include <SX1262Ex.h>

#include "packet.h"
#include "packetParser.h"

namespace pizda {
	using namespace YOBA;
	
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
			constexpr static const char* _logTag = "XCVR";
			constexpr static uint32_t _connectionLostInterval = 5'000'000;
			
			SX1262Ex sx1262 {};
			
			bool _sxSetup = false;
			
			PacketParser* _packetParser = nullptr;
			uint32_t _connectionLostTime = 0;
			TransceiverConnectionState _connectionState = TransceiverConnectionState::initial;

			constexpr static uint16_t _bufferLength = 255;
			uint8_t _buffer[_bufferLength] {};

			void onStart();
			void updateConnectionLostTime();
	};;
}