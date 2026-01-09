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
		controls,
		trim,
		lights,
		baro,
		autopilot,
		motorConfiguration,
		calibrate
	};
	
	enum class RemoteCalibratePacketSystem : uint8_t {
		accelAndGyro,
		mag
	};
	
	class RemoteCalibratePacket {
		public:
			constexpr static uint8_t systemLengthBits = 2;
	};
	
	class RemoteControlsPacket {
		public:
			constexpr static uint8_t motorLengthBits = 12;
	};
	
	class RemoteTrimPacket {
		public:
			constexpr static uint8_t valueLengthBits = RemoteControlsPacket::motorLengthBits;
	};
	
	class RemoteAutopilotPacket {
		public:
			// Speed
			// The Guinness World Record for the fastest RC jet-powered aircraft is 749.221 km/h, set by Niels Herbrich in 2017.
			// In our case 350 km/h (97.2 m/s) will be enough, because it gives precision of ~0.37 m/s per 1 bit
			constexpr static uint8_t speedLengthBits = 8;
			constexpr static int16_t speedMaxMPS = 97;
			
			// 360 deg = 9 bits
			constexpr static uint8_t headingLengthBits = 9;
			
			// Altitude
			// 13 bit = 8191, not enough
			// 14 bit = 16383, more than enough
			// Mapping [-1000; 10000] to [0; 16383]
			constexpr static uint8_t altitudeLengthBits = 14;
			constexpr static int16_t altitudeMinM = -1'000;
			constexpr static int16_t altitudeMaxM = 10'000;
			
			// Mode
			constexpr static uint8_t lateralModeLengthBits = 2;
			constexpr static uint8_t verticalModeLengthBits = 3;
	};
	
	class RemoteBaroPacket {
		public:
			// 1 hectopascal ~= 7.88 meters ~= 30 feet of altitude, which is not good enough for low altitude RC aircraft
			// So we'll be using decapascals. Cool fact: The maximum recorded atmospheric pressure on Earth, adjusted to sea level,
			// is around 1085.7 hPa (32.09 inHg), occurring in Tonsontsengel, Mongolia (2001)
			// So in theory 1100 daPa will be more than enough
			// So 14 bits
			constexpr static uint8_t referencePressureLengthBits = 14;
	};
	
	// -------------------------------- Aircraft --------------------------------
	
	enum class AircraftPacketType : uint8_t {
		ADIRS,
		auxiliary,
		calibration
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
			constexpr static int16_t speedMaxMPS = RemoteAutopilotPacket::speedMaxMPS;
			
			// See RemoteAutopilotPacket
			constexpr static uint8_t altitudeLengthBits = RemoteAutopilotPacket::altitudeLengthBits;
			constexpr static int16_t altitudeMinM = RemoteAutopilotPacket::altitudeMinM;
			constexpr static int16_t altitudeMaxM = RemoteAutopilotPacket::altitudeMaxM;
			
			constexpr static uint8_t autopilotRollLengthBits = AircraftADIRSPacket::rollLengthBits;
			constexpr static float autopilotRollRangeRad = AircraftADIRSPacket::rollRangeRad;
			
			constexpr static uint8_t autopilotPitchLengthBits = AircraftADIRSPacket::pitchLengthBits;
			constexpr static float autopilotPitchRangeRad = AircraftADIRSPacket::pitchRangeRad;
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
			
			// See RemoteAutopilotPacket
			constexpr static uint8_t autopilotLateralModeLengthBits = RemoteAutopilotPacket::lateralModeLengthBits;
			constexpr static uint8_t autopilotVerticalModeLengthBits = RemoteAutopilotPacket::verticalModeLengthBits;
			
			// See RemoteAutopilotPacket
			constexpr static uint8_t autopilotAltitudeLengthBits = RemoteAutopilotPacket::altitudeLengthBits;
			constexpr static int16_t autopilotAltitudeMinM = RemoteAutopilotPacket::altitudeMinM;
			constexpr static int16_t autopilotAltitudeMaxM = RemoteAutopilotPacket::altitudeMaxM;
	};
	
	#pragma pack(pop)
}