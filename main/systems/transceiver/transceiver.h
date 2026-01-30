#pragma once

#include <cstdint>
#include <utility>
#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <SX1262.h>
#include <bitStream.h>
#include "utilities/math.h"

#include "config.h"
#include "types/generic.h"
#include "packet.h"

namespace pizda {
	template<typename TLocalPacketType, typename TRemotePacketType>
	class Transceiver {
		public:
			virtual ~Transceiver() = default;

			bool setup() {
				const auto error = _SX.setup(
					config::spi::device,
					config::transceiver::SPIFrequencyHz,

					config::transceiver::SS,
					config::transceiver::RST,
					config::transceiver::busy,
					config::transceiver::DIO1,

					config::transceiver::RFFrequencyHz,
					config::transceiver::bandwidth,
					config::transceiver::spreadingFactor,
					config::transceiver::codingRate,
					config::transceiver::syncWord,
					config::transceiver::powerDBm,
					config::transceiver::preambleLength
				);

				if (error != SX1262Error::none) {
					ESP_LOGE(_logTag, "SX1262 setup failed with code %d", std::to_underlying(error));
					return false;
				}

				xTaskCreate(
					[](void* arg) {
						static_cast<Transceiver*>(arg)->onStart();
					},
					"XCVR",
					8 * 1024,
					this,
					configMAX_PRIORITIES - 1,
					nullptr
				);

				return true;
			}

			bool receive(const uint32_t timeoutUs) {
				const auto receiveStartTimeUs = esp_timer_get_time();

				uint8_t receivedLength = 0;


				const auto error = _SX.receive(_buffer, receivedLength, timeoutUs);

				if (error == SX1262Error::none) {
					//		ESP_LOGI(_logTag, "read length: %d", length);
					//
					//		for (int i = 0; i < length; ++i) {
					//			ESP_LOGI(_logTag, "read buffer[%d]: %d", i, _buffer[i]);
					//		}

					// Updating RSSI and SNR
					if (esp_timer_get_time() > _RSSIAndSNRUpdateTimeUs) {
						float valueF;

						if (_SX.getRSSI(valueF) == SX1262Error::none)
							_RSSI = static_cast<int8_t>(valueF);

						if (_SX.getSNR(valueF) == SX1262Error::none)
							_SNR = static_cast<int8_t>(valueF);

						_RSSIAndSNRUpdateTimeUs = esp_timer_get_time() + _RSSIAndSNRUpdateIntervalUs;
					}

					// Length check
					if (receivedLength >= 1) {
						const uint8_t payloadLength = receivedLength - Packet::checksumLengthBytes;

						BitStream stream { _buffer };

						// Type
						const auto packetType = static_cast<TRemotePacketType>(stream.readUint8(Packet::typeLengthBits));

						// Reading
						if (onReceive(stream, packetType, payloadLength)) {
							_RXDurationUs = esp_timer_get_time() - receiveStartTimeUs;
							_connectionLostTimeUs = esp_timer_get_time() + _connectionLostIntervalUs;

							if (_connectionState != ConnectionState::connected)
								setConnectionState(ConnectionState::connected);

							return true;
						}
					}
					else {
						ESP_LOGE(_logTag, "failed to receive packet: length %d is too small to fit any data", receivedLength);
					}
				}
				else {
					logSXError("receive error", error);
				}

				if (_connectionState == ConnectionState::connected && esp_timer_get_time() >= _connectionLostTimeUs)
					setConnectionState(ConnectionState::disconnected);

				return false;
			}

			bool transmit(const uint32_t timeoutUs) {
				const auto transmitStartTimeUs = esp_timer_get_time();

				BitStream stream { _buffer };

				const auto packetType = getTransmitPacketType();

				// Type
				stream.writeUint8(static_cast<uint8_t>(packetType), Packet::typeLengthBits);

				// Body
				if (!onTransmit(stream, packetType))
					return false;

				// Checksum
				const auto payloadLength = stream.getBytesProcessed();
				const auto checksum = getCRC8(stream.getBuffer(), payloadLength);
				stream.finishByte();
				stream.writeUint8(checksum, Packet::checksumLengthBytes * 8);

				const auto totalLength = static_cast<uint8_t>(payloadLength + Packet::checksumLengthBytes);

				// Transmitting

				//		ESP_LOGI(_logTag, "write length: %d", length);
				//
				//		for (int i = 0; i < length; ++i) {
				//			ESP_LOGI(_logTag, "write buffer[%d]: %d", i, _buffer[i]);
				//		}

				const auto error = _SX.transmit(_buffer, totalLength, timeoutUs);

				_TXDurationUs = esp_timer_get_time() - transmitStartTimeUs;

				if (error != SX1262Error::none) {
					logSXError("transmit error", error);
					return true;
				}

				return false;
			}

			float getSNR() const {
				return _SNR;
			}

			float getRSSI() const {
				return _RSSI;
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

		protected:
			constexpr static const char* _logTag = "XCVR";
			SX1262 _SX {};

			constexpr static uint16_t _bufferLength = 255;
			uint8_t _buffer[_bufferLength] {};

			static void logSXError(const char* key, const SX1262Error error) {
				if (error == SX1262Error::timeout)
					return;

				constexpr static uint8_t errorBufferLength = 255;
				char errorBuffer[errorBufferLength];

				SX1262::errorToString(error, errorBuffer, errorBufferLength);

				ESP_LOGI(_logTag, "%s: %s", key, errorBuffer);
			}

			static uint8_t getCRC8(const uint8_t* buffer, const size_t length) {
				uint8_t crc = 0xff;

				for (size_t i = 0; i < length; i++) {
					crc ^= buffer[i];

					for (size_t j = 0; j < 8; j++) {
						if ((crc & 0x80) != 0)
							crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
						else
							crc <<= 1;
					}
				}

				return crc;
			}

			static bool validatePayloadChecksumAndLength(BitStream& stream, size_t expectedDataLengthBits, uint8_t payloadLength) {
				const auto expectedPayloadLengthBytes = static_cast<uint8_t>(divideCeiling<size_t>(static_cast<size_t>(Packet::typeLengthBits) + expectedDataLengthBits, 8));

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

			static float readRadians(BitStream& stream, const float range, const uint8_t bits) {
				auto value = static_cast<float>(stream.readUint16(bits)) / static_cast<float>((1 << bits) - 1);
				value = value - 0.5f;
				value = value * range;

				return sanitizeValue<float>(value, toRadians(-180), toRadians(180));
			}

			static void writeRadians(BitStream& stream, const float value, const float range, const uint8_t bits) {
				const auto uintValue = static_cast<uint16_t>((value / range + 0.5f) * ((1 << bits) - 1));

				stream.writeUint16(uintValue, bits);
			}

			static int16_t readAltitude(BitStream& stream, const uint8_t lengthBits, const int16_t min, const int16_t max) {
				const auto factor =
					static_cast<float>(stream.readUint16(lengthBits))
					/ static_cast<float>((1 << lengthBits) - 1);

				return min + (max - min) * factor;
			}

			static void writeAltitude(BitStream& stream, const float valueM, const uint8_t lengthBits, const int16_t min, const int16_t max) {
				const auto factor =
					(std::clamp<float>(valueM, min, max) - static_cast<float>(min))
					/ static_cast<float>(max - min);

				stream.writeUint16(
					static_cast<uint16_t>(std::round(factor * static_cast<float>((1 << lengthBits) - 1))),
					lengthBits
				);
			}

			virtual void onStart() = 0;
			virtual bool onReceive(BitStream& stream, TRemotePacketType packetType, uint8_t payloadLength) = 0;

			virtual TLocalPacketType getTransmitPacketType() = 0;
			virtual bool onTransmit(BitStream& stream, TLocalPacketType packetType) = 0;
			virtual void onConnectionStateChanged() = 0;

		private:
			constexpr static uint8_t _desiredTickFrequencyHz = 30;
			constexpr static uint32_t _desiredTickDurationUs = 1'000'000 / _desiredTickFrequencyHz;
			constexpr static uint32_t _safetyMarginDurationUs = 2'000;

			constexpr static uint32_t _desiredTXDurationUs = (_desiredTickDurationUs - _safetyMarginDurationUs) / 2;
			constexpr static uint32_t _desiredRXDurationUs = _desiredTickDurationUs - _desiredTXDurationUs - _safetyMarginDurationUs;

			constexpr static uint8_t _RSSIAndSNRUpdateFrequencyHz = 1;
			constexpr static uint32_t _RSSIAndSNRUpdateIntervalUs = 1'000'000 / _RSSIAndSNRUpdateFrequencyHz;

			int64_t _RSSIAndSNRUpdateTimeUs = 0;

			int8_t _RSSI = 0;
			int8_t _SNR = 0;

			int64_t _RXDurationUs = 0;
			int64_t _TXDurationUs = 0;

			// ----------------------------- Connection state -----------------------------

			constexpr static uint32_t _connectionLostIntervalUs = 5'000'000;
			int64_t _connectionLostTimeUs = 0;
			ConnectionState _connectionState = ConnectionState::initial;

			void setConnectionState(const ConnectionState state) {
				_connectionState = state;

				onConnectionStateChanged();
			}
	};
}