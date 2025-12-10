#include "transceiver.h"

#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include "freertos/event_groups.h"
#include <freertos/task.h>

#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "aircraft.h"

namespace pizda {
	void Transceiver::setup() {
		QueueHandle_t queue;
		ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, _readingBufferLength, _readingBufferLength, 10, &queue, 0));

		uart_config_t uartConfig {};
		uartConfig.baud_rate = 115200;
		uartConfig.data_bits = UART_DATA_8_BITS;
		uartConfig.parity = UART_PARITY_DISABLE;
		uartConfig.stop_bits = UART_STOP_BITS_1;
		uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
		uartConfig.source_clk = UART_SCLK_DEFAULT;
		ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uartConfig));

		ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	}

	void Transceiver::setPacketParser(PacketParser* value) {
		_packetParser = value;
	}

	void Transceiver::readingTask(void* arg) {
		reinterpret_cast<Transceiver*>(arg)->onReadingTaskTick();
	}

	void Transceiver::onReadingTaskTick() {
		ESP_LOGI("Transceiver", "reading started");

		while (true) {
			const int bytesRead = uart_read_bytes(UART_NUM_0, _readingBuffer, _readingBufferLength, pdMS_TO_TICKS(16));

			if (!_packetParser)
				continue;

			if (bytesRead > 0) {
				ESP_LOGI("Transceiver", "bytes read: %d", bytesRead);

				switch (_connectionState) {
					case TransceiverConnectionState::initial:
						_connectionState = TransceiverConnectionState::normal;
						break;

					case TransceiverConnectionState::lost:
						_connectionState = TransceiverConnectionState::normal;
						_packetParser->onConnectionRestored();
						break;

					default:
						break;
				}

				_packetParser->parse(_readingBuffer, bytesRead);

				updateConnectionLostTime();
			}
			else {
				if (_connectionState == TransceiverConnectionState::normal) {
					if (esp_timer_get_time() >= _connectionLostTime) {
						_connectionState = TransceiverConnectionState::lost;
						_packetParser->onConnectionLost();
					}
				}
			}
		}
	}

	void Transceiver::updateConnectionLostTime() {
		_connectionLostTime = esp_timer_get_time() + _connectionLostInterval;
	}

	void Transceiver::start() {
		xTaskCreate(readingTask, "UARTReadingTask", 4096, this, configMAX_PRIORITIES - 1, nullptr);
	}
}