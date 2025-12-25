#pragma once

#include <cmath>
#include <functional>

#include <bitStream.h>
#include <SX1262Ex.h>

#include "packet.h"
#include "packetHandler.h"

namespace pizda {
	using namespace YOBA;
	
	enum class TransceiverConnectionState : uint8_t {
		initial,
		normal,
		lost
	};

	class Transceiver {
		public:
			bool setup();
			void setPacketHandler(PacketHandler* value);
			void start();
			float getRSSI() const;

		private:
			constexpr static const char* _logTag = "XCVR";
			constexpr static uint32_t _connectionLostIntervalUs = 5'000'000;
			
			constexpr static uint32_t _SXTransmitTimeoutUs = 500'000;
			constexpr static uint32_t _SXReceiveTimeoutUs = 500'000;
			SX1262Ex _SX {};
			
			PacketHandler* _packetHandler = nullptr;
			
			float _RSSI = 0;
			
			bool _reading = false;
			
			constexpr static uint16_t _bufferLength = 255;
			uint8_t _buffer[_bufferLength] {};
			
			uint32_t _connectionLostTime = 0;
			TransceiverConnectionState _connectionState = TransceiverConnectionState::initial;
			
			void updateConnectionLostTime();
			void onStart();
			bool write();
			bool read();
	};
}