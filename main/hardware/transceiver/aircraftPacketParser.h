#pragma once

#include <cmath>
#include <functional>

#include "packetParser.h"

#include <YOBABitStream/main.h>

namespace pizda {
	class AircraftPacketParser : public PacketParser {
		public:

		protected:
			bool onParse(ReadableBitStream& stream, PacketType packetType) override;

		private:
			bool onChannelDataStructurePacket(ReadableBitStream& stream);
			bool onChannelDataPacket(ReadableBitStream& stream);

			bool onMotorConfigurationPacket(ReadableBitStream& stream);
	};
}