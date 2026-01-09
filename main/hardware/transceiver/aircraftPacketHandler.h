#pragma once

#include "packetHandler.h"

#include <cmath>
#include <functional>

#include <esp_log.h>

#include <bitStream.h>

namespace pizda {
	class PacketSequenceItem {
		public:
			PacketSequenceItem(AircraftPacketType type, uint8_t count, bool useEnqueued = false);
			AircraftPacketType getType() const;
			uint8_t getCount() const;
			bool useEnqueued() const;
		
		private:
			AircraftPacketType _type;
			uint8_t _count;
			bool _useEnqueued;
	};
	
	class AircraftPacketHandler : public PacketHandler<AircraftPacketType, RemotePacketType> {
		public:
			void enqueue(AircraftPacketType type) {
				_packetQueue.push(type);
			}
		
		protected:
			[[noreturn]] void onStart() override;
			bool onReceive(BitStream& stream, RemotePacketType packetType, uint8_t payloadLength) override;
			AircraftPacketType getTransmitPacketType() override;
			bool onTransmit(BitStream& stream, AircraftPacketType packetType) override;
			void onIsConnectedChanged() override;
		
		private:
			std::vector<PacketSequenceItem> _packetSequence {
				PacketSequenceItem(AircraftPacketType::ADIRS, 3),
				PacketSequenceItem(AircraftPacketType::ADIRS, 1, true),
				PacketSequenceItem(AircraftPacketType::auxiliary, 1)
			};
			
			uint8_t _packetSequenceIndex = 0;
			uint8_t _packetSequenceItemCounter = 0;
			
			std::queue<AircraftPacketType> _packetQueue {};
			
			bool receiveRemoteControlsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteTrimPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteLightsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteBaroPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteCalibratePacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAutopilotPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveMotorConfigurationPacket(BitStream& stream, uint8_t payloadLength);
			
			void transmitAircraftADIRSPacket(BitStream& stream);
			void transmitAircraftAuxiliaryPacket(BitStream& stream);
			void transmitAircraftCalibrationPacket(BitStream& stream);
	};
}