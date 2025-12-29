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
	
	class AircraftPacketHandler : public PacketHandler {
		public:
			void enqueue(AircraftPacketType type) {
				_packetQueue.push(type);
			}
		
		protected:
			[[noreturn]] void onStart() override;
			bool onReceive(BitStream& stream, uint8_t packetType, uint8_t payloadLength) override;
			uint8_t getTransmitPacketType() override;
			bool onTransmit(BitStream& stream, uint8_t packetType) override;
			void onIsConnectedChanged() override;
		
		private:
			std::vector<PacketSequenceItem> _packetSequence {
				PacketSequenceItem(AircraftPacketType::aircraftADIRS, 4),
				PacketSequenceItem(AircraftPacketType::aircraftAuxiliary, 1),
				PacketSequenceItem(AircraftPacketType::aircraftADIRS, 4),
				PacketSequenceItem(AircraftPacketType::aircraftAutopilot, 1, true)
			};
			
			uint8_t _packetSequenceIndex = 0;
			uint8_t _packetSequenceItemCounter = 0;
			
			std::queue<AircraftPacketType> _packetQueue {};
			
			bool receiveNOPPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteChannelDataStructurePacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteChannelsDataPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveMotorConfigurationPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAutopilotPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryPacket(BitStream& stream, uint8_t payloadLength);
			
			bool transmitAircraftADIRSPacket(BitStream& stream);
			bool transmitAircraftAutopilotPacket(BitStream& stream);
			bool transmitAircraftAuxiliaryPacket(BitStream& stream);
			
			template<typename T>
			static float sanitizeValue(T value, T min, T max) {
				if (value < min) {
					ESP_LOGW(_logTag, "value %f is out of range [%f, %f]", static_cast<float>(value), static_cast<float>(min), static_cast<float>(max));
					
					value = min;
					
				}
				else if (value > max) {
					ESP_LOGW(_logTag, "value %f is out of range [%f, %f]", static_cast<float>(value), static_cast<float>(min), static_cast<float>(max));
					
					value = max;
				}
				
				return value;
			}
			
			static void writeRadians(BitStream& stream, float value, float range, uint8_t bits);
	};
}