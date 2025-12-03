#include "transceiver.h"

#include <cmath>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include "freertos/event_groups.h"
#include <freertos/task.h>

#include <driver/uart.h>
#include <esp_log.h>

#include "rc.h"

namespace pizda {
	void Transceiver::setup() {
		QueueHandle_t queue;
		ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, rxBufferLength * 2, rxBufferLength * 2, 10, &queue, 0));

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
		xTaskCreate(rxTask, "uart_rx_task", 4096, this, configMAX_PRIORITIES - 1, nullptr);
	}

	void Transceiver::rxTask(void* arg) {
		auto instance = reinterpret_cast<Transceiver*>(arg);

		ESP_LOGI("Transceiver", "Rx started");

		while (true) {
			const int bytesRead = uart_read_bytes(UART_NUM_0, instance->rxBuffer, rxBufferLength, 20 / portTICK_PERIOD_MS);

			if (bytesRead > 0) {
				ESP_LOGI("Transceiver", "Got packet, length: %d", bytesRead);

				instance->analyze();
			}
		}
	}

	uint8_t getCRC8(const uint8_t* data, size_t length) {
		uint8_t crc = 0xff;
		size_t i, j;

		for (i = 0; i < length; i++) {
			crc ^= data[i];

			for (j = 0; j < 8; j++) {
				if ((crc & 0x80) != 0)
					crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
				else
					crc <<= 1;
			}
		}

		return crc;
	}

	bool checkCRC(const uint8_t* data, size_t length) {
		auto checksum = getCRC8(data, length);
		auto expectedChecksum = *(data + length);

		ESP_LOGI("Transceiver (packet)", "Checksum: %d, expected: %d", checksum, expectedChecksum);

		if (checksum != expectedChecksum) {
			ESP_LOGE("Transceiver (packet)", "Checksum mismatch: got %d, expected %d", checksum, expectedChecksum);

			return false;
		}

		return true;
	}

	void Transceiver::analyze() {
		auto& rc = RC::getInstance();

		auto packetPtr = rxBuffer;

		// Header
		if (memcmp(packetPtr, PacketExtensions::header, PacketExtensions::headerLength) != 0) {
			ESP_LOGE("Transceiver (packet)", "Mismatched header: %s", packetPtr);

			return;
		}

		packetPtr += PacketExtensions::headerLength;

		// Type
		const auto dataType = *reinterpret_cast<PacketDataType*>(packetPtr);
		packetPtr += 1;

		ESP_LOGI("Transceiver (packet)", "Data type: %d", static_cast<uint8_t>(dataType));

		switch (dataType) {
			case PacketDataType::RemoteControls: {
				if (!checkCRC(packetPtr, sizeof(RemoteControlsPacketData)))
					return;

				auto packetData = reinterpret_cast<RemoteControlsPacketData*>(packetPtr);

				rc.motors.leftAileron.setUint16(packetData->leftAileron);
				rc.motors.leftFlap.setUint16(packetData->flaps);

				ESP_LOGI("Transceiver (packet)", "-------- Lights packet data --------");

				ESP_LOGI("Transceiver (packet)", "Left engine: %d", packetData->leftEngine);
				ESP_LOGI("Transceiver (packet)", "Right engine: %d", packetData->rightEngine);

				ESP_LOGI("Transceiver (packet)", "Left aileron: %d", packetData->leftAileron);
				ESP_LOGI("Transceiver (packet)", "Right aileron: %d", packetData->rightAileron);

				ESP_LOGI("Transceiver (packet)", "Flaps: %d", packetData->flaps);

				ESP_LOGI("Transceiver (packet)", "----------------");

				break;
			}
			case PacketDataType::RemoteLights: {
				if (!checkCRC(packetPtr, sizeof(RemoteLightsPacketData)))
					return;

				auto packetData = reinterpret_cast<RemoteLightsPacketData*>(packetPtr);

				rc.lights.setNavigationEnabled((packetData->value >> 0) & 0b1);
				rc.lights.setStrobeEnabled((packetData->value >> 1) & 0b1);
				rc.lights.setLandingEnabled((packetData->value >> 2) & 0b1);

				ESP_LOGI("Transceiver (packet)", "-------- Lights packet data --------");
				ESP_LOGI("Transceiver (packet)", "Nav lights: %d", rc.lights.isNavigationEnabled());
				ESP_LOGI("Transceiver (packet)", "Strobe lights: %d", rc.lights.isStrobeEnabled());
				ESP_LOGI("Transceiver (packet)", "Landing lights: %d", rc.lights.isLandingEnabled());
				ESP_LOGI("Transceiver (packet)", "----------------");

				break;
			}
			case PacketDataType::Aircraft: {
				break;
			}
		}
	}
}