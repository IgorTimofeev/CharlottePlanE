#pragma once

#include <cmath>

namespace pizda {
	enum class PacketDataType : uint8_t {
		RemoteControls,
		RemoteLights,
		Aircraft
	};

	#pragma pack(push, 1)
		class RemoteControlsPacketData {
			public:
				uint16_t leftEngine;
				uint16_t rightEngine;

				uint16_t leftAileron;
				uint16_t rightAileron;

				uint16_t leftTail;
				uint16_t rightTail;

				uint16_t flaps;
		};
	#pragma pack(pop)

	#pragma pack(push, 1)
		class RemoteLightsPacketData {
			public:
				uint8_t value;
		};
	#pragma pack(pop)

	#pragma pack(push, 1)
		class AircraftPacketData {
			public:
				uint8_t roll;
		};
	#pragma pack(pop)

	class PacketExtensions {
		public:
			constexpr static uint8_t headerLength = 4;
			constexpr static const char* header = "cyka";

			static uint8_t getDataLength(PacketDataType type) {
				switch (type) {
					case PacketDataType::RemoteControls: return sizeof(RemoteControlsPacketData);
					case PacketDataType::RemoteLights: return sizeof(RemoteLightsPacketData);
					case PacketDataType::Aircraft: return sizeof(AircraftPacketData);
				}
			}
	};

	#pragma pack(push, 1)
		template <typename TData>
		class Packet {
			public:
				uint8_t header[PacketExtensions::headerLength];
				PacketDataType type;
				TData data;
				uint8_t checksum;
		};
	#pragma pack(pop)
}