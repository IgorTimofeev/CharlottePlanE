#pragma once

#include <cmath>
#include <functional>

#include <bitStream.h>
#include "utils/math.h"

#include "packet.h"

namespace pizda {
	using namespace YOBA;
	
	enum class TransceiverConnectionState : uint8_t;
	
	class PacketHandler {
		public:
			bool read(uint8_t* buffer, uint8_t length);
			bool write(uint8_t* buffer, PacketType packetType, uint8_t& length);
			
			virtual void onConnectionStateChanged(TransceiverConnectionState fromState, TransceiverConnectionState toState) = 0;
		
		protected:
			constexpr static const char* _logTag = "PacketHandler";
			
			static uint8_t getCRC8(const uint8_t* buffer, size_t length);
			static bool validatePayloadChecksumAndLength(BitStream& stream, size_t dataBitCount, uint8_t payloadLength);
			
			virtual bool readPacket(BitStream& stream, PacketType packetType, uint8_t payloadLength) = 0;
			virtual bool writePacket(BitStream& stream, PacketType packetType) = 0;
	};
}