#pragma once

#include <cmath>
#include "settings.h"

namespace pizda {
	enum class PacketDataType : uint8_t {
		RemoteControlsValues,
		RemoteControlsCalibration,
		RemoteLights,

		Aircraft
	};

	#pragma pack(push, 1)
		struct RemoteControlsValuesPacketData {
			uint16_t leftEngine {};
			uint16_t rightEngine {};

			uint16_t leftAileron {};
			uint16_t rightAileron {};

			uint16_t leftTail {};
			uint16_t rightTail {};

			uint16_t leftFlap {};
			uint16_t rightFlap {};
		};
	#pragma pack(pop)

	#pragma pack(push, 1)
		struct RemoteControlsCalibrationPacketData {
			MotorSettings leftEngine {};
			MotorSettings rightEngine {};

			MotorSettings leftAileron {};
			MotorSettings rightAileron {};

			MotorSettings leftTail {};
			MotorSettings rightTail {};

			MotorSettings leftFlap {};
			MotorSettings rightFlap {};
		};
	#pragma pack(pop)

	#pragma pack(push, 1)
		struct RemoteLightsPacketData {
			uint8_t navigation: 1;
			uint8_t strobe: 1;
			uint8_t landing: 1;
		};
	#pragma pack(pop)

	#pragma pack(push, 1)
		struct AircraftPacketData {
			uint8_t roll = 0;
		};
	#pragma pack(pop)

	class PacketExtensions {
		public:
			constexpr static uint8_t headerLength = 4;
			constexpr static const char* header = "cyka";

			static uint8_t getDataLength(PacketDataType type) {
				switch (type) {
					case PacketDataType::RemoteControlsValues: return sizeof(RemoteControlsValuesPacketData);
					case PacketDataType::RemoteControlsCalibration: return sizeof(RemoteControlsCalibrationPacketData);
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