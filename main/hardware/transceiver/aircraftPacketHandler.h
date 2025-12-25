#pragma once

#include "packetHandler.h"

#include <cmath>
#include <functional>

#include <bitStream.h>

namespace pizda {
	class AircraftPacketHandler : public PacketHandler {
		public:
			void onConnectionLost() override;
			void onConnectionRestored() override;

		protected:
			bool readPacket(BitStream& stream, PacketType packetType, uint8_t payloadLength) override;
			bool writePacket(BitStream& stream, PacketType packetType) override;
			
		private:
			bool readChannelDataStructurePacket(BitStream& stream, uint8_t payloadLength);
			bool readChannelDataPacket(BitStream& stream, uint8_t payloadLength);
			bool readMotorConfigurationPacket(BitStream& stream, uint8_t payloadLength);
			
			bool writeAircraftADIRSPacket(BitStream& stream);
	};
}