#pragma once

#include <cmath>
#include "packet.h"

namespace pizda {
	class Transceiver {
		public:
			void setup() const;

			void start();

			Packet lastPacket {
				PacketType::Aircraft,
				127,
				127,
				127,
				1,
				1,
				0
			};

		private:
			constexpr static uint16_t bufferLength = 2048;

			static void rxTask(void *arg);
	};;
}