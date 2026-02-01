#pragma once

#include <array>

#include "transceiver.h"

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

	class AircraftTransceiver : public Transceiver<
		AircraftPacketType,
		AircraftPacket::typeLengthBits,

		RemotePacketType,
		RemotePacket::typeLengthBits
	> {
		public:
			void enqueueAuxiliary(AircraftAuxiliaryPacketType type);

		protected:
			[[noreturn]] void onStart() override;
			AircraftPacketType getTransmitPacketType() override;
			void onTransmit(BitStream& stream, AircraftPacketType packetType) override;
			bool onReceive(BitStream& stream, RemotePacketType packetType, uint8_t payloadLength) override;
			bool receiveRemoteAuxiliaryPacket(BitStream& stream, uint8_t payloadLength);
			void onConnectionStateChanged() override;

		private:
			constexpr static uint32_t _trendsInterval = 500'000;
			int64_t _trendsTime = 0;
			float _trendsAirspeedPrevMPS = 0;
			float _trendsAltitudePrevM = 0;

			std::array<PacketSequenceItem, 3> _packetSequence {
				PacketSequenceItem(AircraftPacketType::telemetryPrimary, 3),
				PacketSequenceItem(AircraftPacketType::telemetryPrimary, 1, true),
				PacketSequenceItem(AircraftPacketType::telemetrySecondary, 1)
			};

			uint8_t _packetSequenceIndex = 0;
			uint8_t _packetSequenceItemCounter = 0;

			// FIFO packet queue
			int16_t _packetQueueIndex = -1;
			AircraftAuxiliaryPacketType _packetQueue[255] {};
			AircraftAuxiliaryPacketType _auxiliaryPacketType = AircraftAuxiliaryPacketType::calibration;

			bool receiveRemoteControlsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryTrimPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryLightsPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryBaroPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryAutopilotPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryMotorConfigurationPacket(BitStream& stream, uint8_t payloadLength);
			bool receiveRemoteAuxiliaryCalibratePacket(BitStream& stream, uint8_t payloadLength);

			void transmitAircraftTelemetryPrimaryPacket(BitStream& stream);
			void transmitAircraftTelemetrySecondaryPacket(BitStream& stream);
			void transmitAircraftAuxiliaryPacket(BitStream& stream);
			void transmitAircraftAuxiliaryCalibrationPacket(BitStream& stream);
	};
}