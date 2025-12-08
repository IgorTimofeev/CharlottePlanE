#pragma once

#include <cmath>
#include <functional>

#include "packetParser.h"

#include <YOBABitStream/main.h>

namespace pizda {
	class RCPacketParser : public PacketParser {
		public:

		protected:
			bool onParse(ReadableBitStream& stream, PacketType packetType) override;

		private:
			static void onChannelRead(ReadableBitStream& stream, const RemoteChannelDataStructureSettingsField& field, uint8_t channelIndex);

			void onMotorConfiguration(ReadableBitStream& stream, uint8_t motorIndex);
	};
}