#pragma once

#include <cmath>

namespace pizda {
	enum class PacketType : uint8_t {
		Remote,
		Aircraft
	};

	struct Packet {
		PacketType type;
		uint8_t leftWingAileron;
		uint8_t rightWingAileron;
		uint8_t flaps;

		uint8_t navigationLights: 1;
		uint8_t strobeLights: 1;
		uint8_t landingLights: 1;
	};
}