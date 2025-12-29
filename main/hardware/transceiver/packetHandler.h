#pragma once

#include <cstdint>
#include <cmath>
#include <queue>
#include <vector>
#include <optional>
#include <functional>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <bitStream.h>
#include "utils/math.h"

#include "config.h"
#include "packet.h"
#include "hardware/transceiver/SX1262Transceiver.h"

namespace pizda {
	using namespace YOBA;
	
	enum class ConnectionState : uint8_t {
		initial,
		connected,
		disconnected
	};
	
	enum class RemoteState : uint8_t {
		normal,
	};
	
	enum class AircraftState : uint8_t {
		initial,
		aircraftNormal,
		aircraftCalibrating
	};
	
	class PacketHandler {
		public:
			void start() {
				xTaskCreate(
					[](void* arg) {
						reinterpret_cast<PacketHandler*>(arg)->onStart();
					},
					"PacketHandler",
					16 * 1024,
					this,
					10,
					nullptr
				);
			}
			
			bool receive(uint32_t timeoutUs) {
				const auto receiveStartTimeUs = esp_timer_get_time();
				
				uint8_t receivedLength = 0;
				
				// Updating connection state
				auto state = _transceiver->receive(_buffer, receivedLength, timeoutUs);
				
				if (state) {
					switch (_connectionState) {
						case ConnectionState::initial:
							setConnectionState(ConnectionState::connected);
							
							break;
						
						case ConnectionState::disconnected:
							setConnectionState(ConnectionState::connected);
							
							break;
						
						default:
							break;
					}
					
					_connectionLostTime = esp_timer_get_time() + _connectionLostInterval;
					
				}
				else {
					if (_connectionState == ConnectionState::connected) {
						if (esp_timer_get_time() >= _connectionLostTime) {
							setConnectionState(ConnectionState::disconnected);
						}
					}
					
					return false;
				}

//		ESP_LOGI(_logTag, "-------- Begin --------");
				
				// Length check
				if (receivedLength < 1) {
					ESP_LOGE(_logTag, "failed to receive packet: length %d is too small to fit any data", receivedLength);
					
					return false;
				}
				
				const uint8_t payloadLength = receivedLength - Packet::checksumLengthBytes;
				
				BitStream stream { _buffer };
				
				// Type
				const auto packetType = stream.readUint8(Packet::typeLengthBits);
				
				// Reading
				if (!onReceive(stream, packetType, payloadLength))
					return false;
				
				_RXDurationUs = esp_timer_get_time() - receiveStartTimeUs;
				
				return true;
			}
			
			bool transmit(uint32_t timeoutUs) {
				const auto transmitStartTimeUs = esp_timer_get_time();
				
				BitStream stream { _buffer };
				
				const auto packetType = getTransmitPacketType();
				
				// Type
				stream.writeUint8(packetType, Packet::typeLengthBits);
				
				// Body
				if (!onTransmit(stream, packetType))
					return false;
				
				// Checksum
				const auto payloadLength = stream.getBytesProcessed();
				const auto checksum = getCRC8(stream.getBuffer(), payloadLength);
				stream.finishByte();
				stream.writeUint8(checksum, Packet::checksumLengthBytes * 8);
				
				const auto length = static_cast<uint8_t>(payloadLength + Packet::checksumLengthBytes);
				
				// Transmitting
				const auto state = _transceiver->transmit(_buffer, length, timeoutUs);
				
				_TXDurationUs = esp_timer_get_time() - transmitStartTimeUs;
				
				return state;
			}
			
			void setTransceiver(Transceiver* transceiver) {
				_transceiver = transceiver;
			}
			
			const int64_t& getRxDurationUs() const {
				return _RXDurationUs;
			}
			
			const int64_t& getTxDurationUs() const {
				return _TXDurationUs;
			}
			
			ConnectionState getConnectionState() const {
				return _connectionState;
			}
			
			bool isConnected() const {
				return _connectionState == ConnectionState::connected;
			}
			
			RemoteState getRemoteState() const {
				return _remoteState;
			}
			
			AircraftState getAircraftState() const {
				return _aircraftState;
			}
		
		protected:
			constexpr static const char* _logTag = "PacketHandler";
			
			static uint8_t getCRC8(const uint8_t* buffer, size_t length) {
				uint8_t crc = 0xff;
				size_t i, j;
				
				for (i = 0; i < length; i++) {
					crc ^= buffer[i];
					
					for (j = 0; j < 8; j++) {
						if ((crc & 0x80) != 0)
							crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
						else
							crc <<= 1;
					}
				}
				
				return crc;
			}
			
			static bool validatePayloadChecksumAndLength(BitStream& stream, size_t dataBitCount, uint8_t payloadLength) {
				const uint8_t expectedPayloadLengthBytes = divideCeiling<uint8_t>(Packet::typeLengthBits + dataBitCount, 8);
				
				// Length
				if (payloadLength != expectedPayloadLengthBytes) {
					ESP_LOGE(_logTag, "payload length mismatch: got %d, expected %d", payloadLength, expectedPayloadLengthBytes);
					
					return false;
				}
				
				const auto checksum = getCRC8(stream.getBuffer(), expectedPayloadLengthBytes);
				const auto expectedChecksum = *(stream.getBuffer() + expectedPayloadLengthBytes);
				
				// Checksum
				if (checksum != expectedChecksum) {
					ESP_LOGE(_logTag, "checksum mismatch: got %d, expected %d", checksum, expectedChecksum);
					
					return false;
				}
				
				return true;
			}
			
			void setRemoteState(RemoteState remoteState) {
				_remoteState = remoteState;
			}
			
			void setAircraftState(AircraftState aircraftState) {
				_aircraftState = aircraftState;
			}
			
			virtual void onStart() = 0;
			virtual bool onReceive(BitStream& stream, uint8_t packetType, uint8_t payloadLength) = 0;
			
			virtual uint8_t getTransmitPacketType() = 0;
			virtual bool onTransmit(BitStream& stream, uint8_t packetType) = 0;
			virtual void onIsConnectedChanged() = 0;
		
		private:
			Transceiver* _transceiver = nullptr;
			
			constexpr static uint16_t _bufferLength = 255;
			uint8_t _buffer[_bufferLength] {};
			
			// ----------------------------- XCVR -----------------------------
			
			constexpr static uint8_t _desiredTickFrequencyHz = 30;
			constexpr static uint32_t _desiredTickDurationUs = 1'000'000 / _desiredTickFrequencyHz;
			constexpr static uint32_t _safetyMarginDurationUs = 2'000;
			
			constexpr static uint32_t _desiredTXDurationUs = (_desiredTickDurationUs - _safetyMarginDurationUs) / 2;
			constexpr static uint32_t _desiredRXDurationUs = _desiredTickDurationUs - _desiredTXDurationUs - _safetyMarginDurationUs;
			
			int64_t _RXDurationUs = 0;
			int64_t _TXDurationUs = 0;
			
			// ----------------------------- Connection state -----------------------------
			
			constexpr static uint32_t _connectionLostInterval = 5'000'000;
			uint32_t _connectionLostTime = 0;
			ConnectionState _connectionState = ConnectionState::initial;
			
			void setConnectionState(ConnectionState state) {
				_connectionState = state;
				
				onIsConnectedChanged();
			}
			
			// ----------------------------- Remote/aircraft state -----------------------------
			
			RemoteState _remoteState = RemoteState::normal;
			AircraftState _aircraftState = AircraftState::initial;
		
	};
}