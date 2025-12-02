#include "transceiver.h"
#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include "freertos/event_groups.h"
#include <freertos/task.h>

#include <driver/uart.h>
#include <esp_log.h>

namespace pizda {
	void Transceiver::setup() const {
		QueueHandle_t uart_queue;
		ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, bufferLength, bufferLength, 10, &uart_queue, 0));

		uart_config_t uart_config {};
		uart_config.baud_rate = 115200;
		uart_config.data_bits = UART_DATA_8_BITS;
		uart_config.parity    = UART_PARITY_DISABLE;
		uart_config.stop_bits = UART_STOP_BITS_1;
		uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
		uart_config.source_clk = UART_SCLK_DEFAULT;
		ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

		ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	}

	void Transceiver::start() {
		xTaskCreate(rxTask, "uart_rx_task", 4096, this, configMAX_PRIORITIES - 1, NULL);
	}

	void Transceiver::rxTask(void* arg) {
		auto instance = reinterpret_cast<Transceiver*>(arg);

		auto data = (uint8_t*) malloc(bufferLength + 1);

		ESP_LOGI("Transceiver", "Rx started, buffer ptr: %p", data);

		while (true) {
			const int bytesRead = uart_read_bytes(UART_NUM_0, data, bufferLength, 20 / portTICK_PERIOD_MS);

			if (bytesRead > 0) {
				ESP_LOGI("Transceiver", "Got packet, length: %d", bytesRead);

				auto packet = reinterpret_cast<Packet*>(data);
				memcpy(&instance->lastPacket, packet, sizeof(Packet));

				ESP_LOGI("Transceiver", "Data: %d", packet->leftWingAileron);
			}
		}
	}
}