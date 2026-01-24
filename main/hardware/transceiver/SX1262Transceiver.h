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
			
			bool transmit(const uint8_t* buffer, const uint8_t length, const uint32_t timeoutUs) override {
//		ESP_LOGI(_logTag, "write length: %d", length);
//
//		for (int i = 0; i < length; ++i) {
//			ESP_LOGI(_logTag, "write buffer[%d]: %d", i, _buffer[i]);
//		}

				const auto error = _SX.transmit(buffer, length, timeoutUs);

				if (error == SX1262Error::none)
					return true;

				logError("transmit error", error);

				return false;
			}
			
			bool receive(uint8_t* buffer, uint8_t& length, const uint32_t timeoutUs) override {
				const auto error = _SX.receive(buffer, length, timeoutUs);

				if (error == SX1262Error::none) {
					//		ESP_LOGI(_logTag, "read length: %d", length);
					//
					//		for (int i = 0; i < length; ++i) {
					//			ESP_LOGI(_logTag, "read buffer[%d]: %d", i, _buffer[i]);
					//		}
					if (esp_timer_get_time() > _RSSIAndSNRUpdateTimeUs) {
						_SX.getRSSI(_RSSI);
						_SX.getSNR(_SNR);

						_RSSIAndSNRUpdateTimeUs = esp_timer_get_time() + _RSSIAndSNRUpdateIntervalUs;
					}

					return true;
				}

				logError("receive error", error);

				return false;
			}
		
		private:
			constexpr static auto _logTag = "XCVR";
			
			constexpr static uint8_t _RSSIAndSNRUpdateFrequencyHz = 1;
			constexpr static uint32_t _RSSIAndSNRUpdateIntervalUs = 1'000'000 / _RSSIAndSNRUpdateFrequencyHz;
			int64_t _RSSIAndSNRUpdateTimeUs = 0;
			
			SX1262 _SX {};
			float _RSSI = 0;
			float _SNR = 0;

			static void logError(const char* key, const SX1262Error error) {
				if (error == SX1262Error::timeout)
					return;

				constexpr static uint8_t errorBufferLength = 255;
				char errorBuffer[errorBufferLength];

				SX1262::errorToString(error, errorBuffer, errorBufferLength);

				ESP_LOGI(_logTag, "%s: %s", key, errorBuffer);
			}
	};
}