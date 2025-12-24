#include "transceiver.h"

#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "freertos/event_groups.h"

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "config.h"
#include "aircraft.h"

namespace pizda {
	void Transceiver::setup() {
		_sxSetup = sx1262.setup(
			config::spi::device,
			config::transceiver::SPIFrequencyHz,
			
			config::transceiver::SS,
			config::common::RST,
			config::transceiver::busy,
			config::transceiver::DIO1,
			
			config::transceiver::RFFrequencyMHz,
			config::transceiver::bandwidthKHz,
			config::transceiver::spreadingFactor,
			config::transceiver::codingRate,
			config::transceiver::syncWord,
			config::transceiver::powerDBm,
			config::transceiver::preambleLength
		);
		
		if (!_sxSetup) {
			ESP_LOGE("Main", "SX1262 setup failed");
		}
	}
	
	void Transceiver::start() {
		xTaskCreate(
			[](void* arg) {
				reinterpret_cast<Transceiver*>(arg)->onStart();
			},
			"Transceiver",
			4096,
			this,
			configMAX_PRIORITIES - 1,
			nullptr
		);
	}
	
	void Transceiver::setPacketParser(PacketParser* value) {
		_packetParser = value;
	}

	void Transceiver::onStart() {
		auto& ac = Aircraft::getInstance();
		
		ESP_LOGI(_logTag, "started");

		while (true) {
			if (!_packetParser || !_sxSetup)
				continue;
			
			// Clearing buffer
			std::memset(_buffer, 0, _bufferLength);
			
			// Copying header
			std::memcpy(_buffer, &Packet::header, Packet::headerLengthBytes);
			
			BitStream stream { _buffer + Packet::headerLengthBytes };

			// Packet type
			stream.writeUint8(static_cast<uint8_t>(PacketType::AircraftAHRS), Packet::typeLengthBits);
			
			// Payload
			ESP_LOGI(_logTag, "rollRad: %f", ac.ahrs.getRollRad());
			ESP_LOGI(_logTag, "pitchRad: %f", ac.ahrs.getPitchRad());
			ESP_LOGI(_logTag, "yawRad: %f", ac.ahrs.getYawRad());
			ESP_LOGI(_logTag, "altitudeM: %f", ac.ahrs.getAltitudeM());
			
			stream.writeFloat(ac.ahrs.getRollRad());
			stream.writeFloat(ac.ahrs.getPitchRad());
			stream.writeFloat(ac.ahrs.getYawRad());
			stream.writeFloat(ac.ahrs.getAltitudeM());
			
			// Transmitting
			const auto packetLength = Packet::headerLengthBytes + stream.getBytesProcessed();
			
			ESP_LOGI(_logTag, "packetLength: %d", packetLength);
			
			if (sx1262.transmit(_buffer, packetLength, 1'000'000)) {
			
			}
			else {
				ESP_LOGE(_logTag, "transmit failed");
			}
			
			vTaskDelay(pdMS_TO_TICKS(1'000));
			
//			if (bytesRead > 0) {
//				ESP_LOGI(_logTag, "bytes readRegister: %d", bytesRead);
//
//				switch (_connectionState) {
//					case TransceiverConnectionState::initial:
//						_connectionState = TransceiverConnectionState::normal;
//						break;
//
//					case TransceiverConnectionState::lost:
//						_connectionState = TransceiverConnectionState::normal;
//						_packetParser->onConnectionRestored();
//						break;
//
//					default:
//						break;
//				}
//
//				_packetParser->parse(_readingBuffer, bytesRead);
//
//				updateConnectionLostTime();
//			}
//			else {
//				if (_connectionState == TransceiverConnectionState::normal) {
//					if (esp_timer_get_time() >= _connectionLostTime) {
//						_connectionState = TransceiverConnectionState::lost;
//						_packetParser->onConnectionLost();
//					}
//				}
//			}
		}
	}

	void Transceiver::updateConnectionLostTime() {
		_connectionLostTime = esp_timer_get_time() + _connectionLostInterval;
	}
}