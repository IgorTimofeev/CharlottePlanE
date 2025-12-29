#pragma once

#include <cstdint>

namespace pizda {
	#pragma pack(push, 1)
	
	class Packet {
		public:
			constexpr static uint8_t typeLengthBits = 4;
			constexpr static uint8_t checksumLengthBytes = 1;
	};
	
	// -------------------------------- Remote --------------------------------
	
	enum class RemotePacketType : uint8_t {
		NOP,
		remoteChannelsDataStructure,
		remoteChannelsData,
		remoteMotorConfiguration,
		remoteAuxiliary,
		remoteAutopilot,
		remoteCalibrate
	};
	
	enum class RemoteCalibratePacketSystem : uint8_t {
		accelAndGyro,
		mag
	};
	
	class RemoteCalibratePacket {
		public:
			constexpr static uint8_t systemLengthBits = 2;
	};
	
	class RemoteChannelsPacket {
		public:
			constexpr static uint8_t motorLengthBits = 9;
	};
	
	class RemoteAuxiliaryPacket {
		public:
			constexpr static uint8_t referencePressureLengthBits = 14;
	};
	
	class RemoteAutopilotPacket {
		public:
			// Speed
			// 7 bit = 127 m/s or ~246.8 kt, should be enough
			constexpr static uint8_t speedLengthBits = 7;
			
			// 360 deg = 9 bits
			constexpr static uint8_t headingLengthBits = 9;
			
			// Altitude
			// 13 bit = 8191, not enough
			// 14 bit = 16383, more than enough
			// Mapping [-1000; 10000] to [0; 16383]
			constexpr static uint8_t altitudeLengthBits = 14;
			constexpr static int16_t altitudeMin = -1'000;
			constexpr static int16_t altitudeMax = 10'000;
	};
	
	// -------------------------------- Aircraft --------------------------------
	
	enum class AircraftPacketType : uint8_t {
		aircraftADIRS,
		aircraftAuxiliary,
		aircraftAutopilot,
		aircraftCalibration
	};
	
	class AircraftADIRSPacket {
		public:
			// Roll / pitch / yaw
			// Precision of 0.25 - 0.5 deg should be enough for any client-side visualization with LPF
			// So 360 * 1 / 0.25 = 1440 ~= 10 bits
			
			// Roll range is [-180; 180] deg
			constexpr static uint8_t rollLengthBits = 10;
			constexpr static float rollRangeRad = 2 * std::numbers::pi_v<float>;
			
			// Pitch is [-90; 90] deg, but we can't use fewer bits
			constexpr static uint8_t pitchLengthBits = 10;
			constexpr static float pitchRangeRad = std::numbers::pi_v<float>;
			
			// Same as roll, [-180; 180] deg
			constexpr static uint8_t yawLengthBits = 10;
			constexpr static float yawRangeRad = 2 * std::numbers::pi_v<float>;
			
			// Slip & skid
			// 8 bit = [-127; 127]
			constexpr static uint8_t slipAndSkidLengthBits = 8;
			constexpr static uint8_t slipAndSkidMaxG = 2;
			
			// See RemoteAutopilotPacket
			constexpr static uint8_t speedLengthBits = RemoteAutopilotPacket::speedLengthBits;
			
			// See RemoteAutopilotPacket
			constexpr static uint8_t altitudeLengthBits = RemoteAutopilotPacket::altitudeLengthBits;
			constexpr static int16_t altitudeMin = RemoteAutopilotPacket::altitudeMin;
			constexpr static int16_t altitudeMax = RemoteAutopilotPacket::altitudeMax;
	};
	
	enum class AircraftCalibrationPacketSystem : uint8_t {
		accelAndGyro,
		mag
	};
	
	class AircraftCalibrationPacket {
		public:
			constexpr static uint8_t systemLengthBits = RemoteCalibratePacket::systemLengthBits;
			constexpr static uint8_t progressLengthBits = 8;
	};
	
	class AircraftAuxiliaryPacket {
		public:
			constexpr static uint8_t throttleLengthBits = 7;
			constexpr static uint8_t latLengthBits = 25;
			constexpr static uint8_t lonLengthBits = 26;
			constexpr static uint8_t batteryLengthBits = 9;
	};
	
	class AircraftAutopilotPacket {
		public:
			constexpr static uint8_t flightDirectorRollLengthBits = AircraftADIRSPacket::rollLengthBits;
			constexpr static float flightDirectorRollRangeRad = AircraftADIRSPacket::rollRangeRad;
			
			constexpr static uint8_t flightDirectorPitchLengthBits = AircraftADIRSPacket::pitchLengthBits;
			constexpr static float flightDirectorPitchRangeRad = AircraftADIRSPacket::pitchRangeRad;
	};
	
	#pragma pack(pop)
}