#pragma once

#include <cstdint>

namespace YOBA {
	class Transceiver {
		public:
			virtual ~Transceiver() = default;
			virtual bool transmit(const uint8_t* buffer, uint8_t length, uint32_t timeoutUs) = 0;
			virtual bool receive(uint8_t* buffer, uint8_t& length, uint32_t timeoutUs) = 0;
	};
}