#pragma once

#include <cmath>
#include "packet.h"

namespace pizda {
	class Transceiver {
		public:
			void setup();
			void start();

		private:
			constexpr static uint16_t rxBufferLength = 255;

			uint8_t rxBuffer[rxBufferLength];

			static void rxTask(void *arg);
			void parsePacket();
	};;
}