#pragma once

#include <cmath>
#include <functional>

#include "packetParser.h"

#include <YOBABitStream/main.h>

namespace pizda {
	class AircraftPacketParser : public PacketParser {
		public:
			void onConnectionLost() override;
			void onConnectionRestored() override;

		protected:
			bool onParse(BitStream& stream, PacketType packetType) override;

		private:
			bool onChannelDataStructurePacket(BitStream& stream);
			bool onChannelDataPacket(BitStream& stream);
			bool onMotorConfigurationPacket(BitStream& stream);
	};
}