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
		standby,
		connected,
		disconnected
	};
	
	class Transceiver {
		public:
			bool setup();
			void setPacketHandler(PacketHandler* value);
			void start();
			float getRSSI() const;
		
		private:
			constexpr static const char* _logTag = "XCVR";
			
			constexpr static uint8_t _desiredTickFrequencyHz = 20;
			constexpr static uint32_t _desiredTickDurationUs = 1'000'000 / _desiredTickFrequencyHz;
			constexpr static uint32_t _safetyMarginDurationUs = 2'000;
			
			constexpr static uint32_t _TXDurationUs = (_desiredTickDurationUs - _safetyMarginDurationUs) / 2;
			constexpr static uint32_t _RXDurationUs = _desiredTickDurationUs - _TXDurationUs - _safetyMarginDurationUs;
			
			SX1262Ex _SX {};
			float _RSSI = 0;
			
			PacketHandler* _packetHandler = nullptr;
			
			constexpr static uint16_t _bufferLength = 255;
			uint8_t _buffer[_bufferLength] {};
			
			constexpr static uint32_t _connectionLostInterval = 5'000'000;
			uint32_t _connectionLostTime = 0;
			TransceiverConnectionState _connectionState = TransceiverConnectionState::standby;
			
			[[noreturn]] void onStart();
			bool transmit(uint32_t timeoutUs);
			bool receive(uint32_t timeoutUs);
			
			void setConnectionState(TransceiverConnectionState state);
	};
}