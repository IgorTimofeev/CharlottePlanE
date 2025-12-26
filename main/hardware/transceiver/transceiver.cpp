#include "transceiver.h"

#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "config.h"

namespace pizda {
	bool Transceiver::setup(bool isSlave) {
		_isSlave = isSlave;
		
		auto state = _SX.setup(
			config::spi::device,
			config::transceiver::SPIFrequencyHz,
			
			config::transceiver::SS,
			config::transceiver::RST,
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
		
		if (!state) {
			ESP_LOGE(_logTag, "SX1262 setup failed");
			
			return false;
		}
		
		return true;
	}
	
	void Transceiver::setPacketHandler(PacketHandler* value) {
		_packetHandler = value;
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
	
	float Transceiver::getRSSI() const {
		return _RSSI;
	}
	
	[[noreturn]] void Transceiver::onStart() {
		ESP_LOGI(_logTag, "started");
		
		if (_isSlave) {
			while (true) {
				if (receive(1'000'000)) {
					vTaskDelay(pdMS_TO_TICKS(20));
					transmit(1'000'000);
				}
			}
		}
		else {
			while (true) {
//				auto time = esp_timer_get_time();
				transmit(500'000);
//				auto txTime = esp_timer_get_time() - time;

//				time = esp_timer_get_time();
				receive(500'000);
//				auto rxTime = esp_timer_get_time() - time;

//				ESP_LOGI("PIZDA", "total time: %d, rx time: %d, txTime: %d", rxTime +  txTime, rxTime, txTime);
			}
		}
	}
	
	bool Transceiver::transmit(uint32_t timeoutUs) {
		uint8_t length = 0;
		
		if (!_packetHandler->write(_buffer, PacketType::aircraftADIRS, length))
			return false;

//		ESP_LOGI(_logTag, "write length: %d", length);
//
//		for (int i = 0; i < length; ++i) {
//			ESP_LOGI(_logTag, "write buffer[%d]: %d", i, _buffer[i]);
//		}
		
		return _SX.transmit(_buffer, length, timeoutUs);
	}
	
	bool Transceiver::receive(uint32_t timeoutUs) {
		uint8_t length = 0;
		
		auto state = _SX.receive(_buffer, length, timeoutUs);

//		ESP_LOGI(_logTag, "read length: %d", length);
//
//		for (int i = 0; i < length; ++i) {
//			ESP_LOGI(_logTag, "read buffer[%d]: %d", i, _buffer[i]);
//		}

		if (state) {
			state = _packetHandler->read(_buffer, length);
		}
		
		if (state) {
			switch (_connectionState) {
				case TransceiverConnectionState::standby:
					setConnectionState(TransceiverConnectionState::connected);
					
					break;
				
				case TransceiverConnectionState::disconnected:
					setConnectionState(TransceiverConnectionState::connected);
					
					break;
				
				default:
					break;
			}
			
			_connectionLostTime = esp_timer_get_time() + _connectionLostInterval;
			
			// RSSI
			_SX.getRSSI(_RSSI);
		}
		else {
			if (_connectionState == TransceiverConnectionState::connected) {
				if (esp_timer_get_time() >= _connectionLostTime) {
					setConnectionState(TransceiverConnectionState::disconnected);
				}
			}
		}
		
		return state;
	}
	
	void Transceiver::setConnectionState(TransceiverConnectionState state) {
		auto oldState = _connectionState;
		_connectionState = state;
		
		_packetHandler->onConnectionStateChanged(oldState, _connectionState);
	}
}