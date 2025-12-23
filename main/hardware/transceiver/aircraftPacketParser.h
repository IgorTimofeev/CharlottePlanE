#pragma once

#include "packetParser.h"

#include <cmath>
#include <functional>

#include <bitStream.h>

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