#pragma once

#include "cstdint"

namespace pizda {
	#pragma pack(push, 1)
	
	enum class PacketType : uint8_t {
		remoteChannelDataStructure,
		remoteChannelData,
		remoteMotorConfiguration,
		remoteBaro,
		
		aircraftADIRS,
		aircraftStatistics,
		aircraftAutopilot
	};
	
	class Packet {
		public:
			constexpr static uint8_t typeLengthBits = 4;
			constexpr static uint8_t checksumLengthBytes = 1;
	};
	
	class AircraftADIRSPacket {
		public:
			// Roll / pitch / yaw
			// Precision of 0.25 - 0.5 deg should be enough for any client-side visualization with LPF
			// So 360 * 1 / 0.25 = 1440 ~= 10 bits
			constexpr static uint8_t rollLengthBits = 10;
			constexpr static uint8_t pitchLengthBits = 10;
			constexpr static uint8_t yawLengthBits = 10;
			
			// Slip & skid
			// 8 bit = [-127; 127] ~= [-2G; 2G]
			constexpr static uint8_t slipAndSkidLengthBits = 8;
			constexpr static uint8_t slipAndSkidMaxG = 2;
			
			// Speed
			// 7 bit = 127 m/s or ~246.8 kt, should be enough
			constexpr static uint8_t speedLengthBits = 7;
			
			// Altitude
			// 13 bit = 8191, not enough
			// 14 bit = 16383, more than enough
			// Mapping [-1000; 10000] to [0; 16383]
			constexpr static uint8_t altitudeLengthBits = 14;
			constexpr static int16_t altitudeMin = -1'000;
			constexpr static int16_t altitudeMax = 10'000;
	};
	
	class AircraftStatisticsPacket {
		public:
			constexpr static uint8_t throttleLengthBits = 7;
			constexpr static uint8_t latLengthBits = 25;
			constexpr static uint8_t lonLengthBits = 26;
			constexpr static uint8_t batteryLengthBits = 9;
	};
	
	#pragma pack(pop)
}