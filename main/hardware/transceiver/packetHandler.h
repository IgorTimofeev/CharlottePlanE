#pragma once

#include <cmath>
#include <queue>
#include <vector>
#include <optional>

#include <bitStream.h>
#include "utils/math.h"

#include "packet.h"

namespace pizda {
	using namespace YOBA;
	
	enum class TransceiverConnectionState : uint8_t;
	
	class PacketHandler {
		public:
			PacketHandler(const std::initializer_list<std::optional<PacketType>>& packetSequence);
			
			bool read(uint8_t* buffer, uint8_t length);
			bool writeNext(uint8_t* buffer, uint8_t& length);
			void enqueue(PacketType type);
			
			virtual void onConnectionStateChanged(TransceiverConnectionState fromState, TransceiverConnectionState toState) = 0;
		
		protected:
			constexpr static const char* _logTag = "PacketHandler";
			
			std::vector<std::optional<PacketType>> _packetSequence;
			uint8_t _packetSequenceIndex = 0;
			
			std::queue<PacketType> _packetQueue {};
			
			static uint8_t getCRC8(const uint8_t* buffer, size_t length);
			static bool validatePayloadChecksumAndLength(BitStream& stream, size_t dataBitCount, uint8_t payloadLength);
			
			virtual bool readPacket(BitStream& stream, PacketType packetType, uint8_t payloadLength) = 0;
			virtual bool writePacket(BitStream& stream, PacketType packetType) = 0;
			
		private:
			PacketType getNextPacketType();
			
			void nextPacketQueueIndex();
	};
}