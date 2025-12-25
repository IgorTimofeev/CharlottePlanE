#include "transceiver.h"

#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_timer.h>

#include "config.h"
#include "aircraft.h"

namespace pizda {
	bool Transceiver::setup() {
		_reading = false;
		
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

	void Transceiver::onStart() {
		ESP_LOGI(_logTag, "started");

		while (true) {
			if (!_packetHandler)
				continue;
			
			bool state = false;
			
			if (_reading) {
				state = read();
				
				if (state) {
					switch (_connectionState) {
						case TransceiverConnectionState::initial:
							_connectionState = TransceiverConnectionState::normal;
							
							break;
						
						case TransceiverConnectionState::lost:
							_connectionState = TransceiverConnectionState::normal;
							_packetHandler->onConnectionRestored();
							
							break;
						
						default:
							break;
					}
					
					updateConnectionLostTime();
					
					// RSSI
					_SX.getRSSI(_RSSI);
				}
				else {
					if (_connectionState == TransceiverConnectionState::normal) {
						if (esp_timer_get_time() >= _connectionLostTime) {
							_connectionState = TransceiverConnectionState::lost;
							_packetHandler->onConnectionLost();
						}
					}
				}
			}
			else {
				state = write();
			}

			vTaskDelay(pdMS_TO_TICKS(1'000 / config::transceiver::tickRateHz));
//			vTaskDelay(pdMS_TO_TICKS(1'000));
		}
	}
	
	void Transceiver::updateConnectionLostTime() {
		_connectionLostTime = esp_timer_get_time() + _connectionLostInterval;
	}
	
	bool Transceiver::write() {
		uint8_t length = 0;
		
		if (!_packetHandler->write(_buffer, PacketType::aircraftADIRS, length))
			return false;
		
//		ESP_LOGI(_logTag, "write length: %d", length);
//
//		for (int i = 0; i < length; ++i) {
//			ESP_LOGI(_logTag, "write buffer[%d]: %d", i, _buffer[i]);
//		}

		if (!_SX.transmit(_buffer, length, 1'000'000))
			return false;
		
		return true;
	}
	
	bool Transceiver::read() {
		uint8_t length = 0;
		
		if (!_SX.receive(_buffer, length, 1'000'000))
			return false;
		
//		ESP_LOGI(_logTag, "read length: %d", length);
//
//		for (int i = 0; i < length; ++i) {
//			ESP_LOGI(_logTag, "read buffer[%d]: %d", i, _buffer[i]);
//		}
		
		if (!_packetHandler->read(_buffer, length))
			return false;
		
		return true;
	}
}