#pragma once

#include <cstdint>
#include <cmath>
#include <utility>

#include <esp_log.h>
#include <esp_timer.h>

#include <SX1262.h>

#include "hardware/transceiver/transceiver.h"
#include "config.h"

namespace pizda {
	using namespace YOBA;
	
	class SX1262Transceiver : public Transceiver {
		public:
			bool setup() {
				const auto error = _SX.setup(
					config::spi::device,
					config::transceiver::SPIFrequencyHz,
					
					config::transceiver::SS,
					config::transceiver::RST,
					config::transceiver::busy,
					config::transceiver::DIO1,
					
					config::transceiver::RFFrequencyMHz,
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
				
				return true;
			}
			
			float getSNR() const {
				return _SNR;
			}
			
			float getRSSI() const {
				return _RSSI;
			}
			
			bool transmit(const uint8_t* buffer, uint8_t length, uint32_t timeoutUs) override {
//		ESP_LOGI(_logTag, "write length: %d", length);
//
//		for (int i = 0; i < length; ++i) {
//			ESP_LOGI(_logTag, "write buffer[%d]: %d", i, _buffer[i]);
//		}
				
				return _SX.transmit(buffer, length, timeoutUs) == SX1262Error::none;
			}
			
			bool receive(uint8_t* buffer, uint8_t& length, uint32_t timeoutUs) override {
				const auto state = _SX.receive(buffer, length, timeoutUs) == SX1262Error::none;
//		ESP_LOGI(_logTag, "read length: %d", length);
//
//		for (int i = 0; i < length; ++i) {
//			ESP_LOGI(_logTag, "read buffer[%d]: %d", i, _buffer[i]);
//		}
				if (state) {
					if (esp_timer_get_time() > _RSSIAndSNRUpdateTimeUs) {
						_SX.getRSSI(_RSSI);
						_SX.getSNR(_SNR);
						
						_RSSIAndSNRUpdateTimeUs = esp_timer_get_time() + _RSSIAndSNRUpdateIntervalUs;
					}
				}
				
				return state;
			}
		
		private:
			constexpr static const char* _logTag = "XCVR";
			
			constexpr static uint8_t _RSSIAndASNRUpdateFrequencyHz = 1;
			constexpr static uint32_t _RSSIAndSNRUpdateIntervalUs = 1'000'000 / _RSSIAndASNRUpdateFrequencyHz;
			int64_t _RSSIAndSNRUpdateTimeUs = 0;
			
			SX1262 _SX {};
			float _RSSI = 0;
			float _SNR = 0;
	};
}