#pragma once

#include <cstdint>

#include <SX1262Ex.h>

namespace pizda {
	class Transceiver {
		public:
			virtual bool transmit(const uint8_t* buffer, uint8_t length, uint32_t timeoutUs) = 0;
			virtual bool receive(uint8_t* buffer, uint8_t& length, uint32_t timeoutUs) = 0;
	};
}