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

				instance->parsePacket();
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

//		ESP_LOGI("Transceiver", "Checksum: %d, expected: %d", checksum, expectedChecksum);

		if (checksum != expectedChecksum) {
			ESP_LOGE("Transceiver", "Checksum mismatch: got %d, expected %d", checksum, expectedChecksum);

			return false;
		}

		return true;
	}

	void Transceiver::parsePacket() {
		auto& rc = RC::getInstance();

		auto packetPtr = rxBuffer;

		// Header
		if (memcmp(packetPtr, PacketExtensions::header, PacketExtensions::headerLength) != 0) {
			ESP_LOGE("Transceiver", "Mismatched header: %s", packetPtr);

			return;
		}

		packetPtr += PacketExtensions::headerLength;

		// Type
		const auto dataType = *reinterpret_cast<PacketDataType*>(packetPtr);
		packetPtr += 1;

		ESP_LOGI("Transceiver", "Data type: %d", static_cast<uint8_t>(dataType));

		switch (dataType) {
			case PacketDataType::RemoteControlsValues: {
				if (!checkCRC(packetPtr, sizeof(RemoteControlsValuesPacketData)))
					return;

				auto packetData = reinterpret_cast<RemoteControlsValuesPacketData*>(packetPtr);

				// Updating motors position
				rc.motors.setLeftAileron(packetData->leftAileron);
				rc.motors.setLeftFlap(packetData->leftFlap);

				ESP_LOGI("Transceiver", "-------- Controls values --------");
				ESP_LOGI("Transceiver", "Left engine: %d", packetData->leftEngine);
				ESP_LOGI("Transceiver", "Right engine: %d", packetData->rightEngine);
				ESP_LOGI("Transceiver", "Left aileron: %d", packetData->leftAileron);
				ESP_LOGI("Transceiver", "Right aileron: %d", packetData->rightAileron);
				ESP_LOGI("Transceiver", "Left tail: %d", packetData->leftTail);
				ESP_LOGI("Transceiver", "Right tail: %d", packetData->rightTail);
				ESP_LOGI("Transceiver", "Left flap: %d", packetData->leftFlap);
				ESP_LOGI("Transceiver", "Right flap: %d", packetData->rightFlap);
				ESP_LOGI("Transceiver", "----------------");

				break;
			}
			case PacketDataType::RemoteControlsCalibration: {
				if (!checkCRC(packetPtr, sizeof(RemoteControlsCalibrationPacketData)))
					return;

				auto packetData = reinterpret_cast<RemoteControlsCalibrationPacketData*>(packetPtr);

				// Saving new calibration data
				rc.settings.motors.leftEngine = packetData->leftEngine;
				rc.settings.motors.rightEngine = packetData->rightEngine;

				rc.settings.motors.leftAileron = packetData->leftAileron;
				rc.settings.motors.rightAileron = packetData->rightAileron;

				rc.settings.motors.leftTail = packetData->leftTail;
				rc.settings.motors.rightTail = packetData->rightTail;

				rc.settings.motors.leftFlap = packetData->leftFlap;
				rc.settings.motors.rightFlap = packetData->rightFlap;

				rc.settings.motors.scheduleWrite();

				// Updating motors position
				rc.motors.setLeftAileron(rc.motors.getLeftAileron());
				rc.motors.setLeftFlap(rc.motors.getLeftFlap());

				ESP_LOGI("Transceiver", "-------- Controls calibration --------");
				ESP_LOGI("Transceiver", "Left engine min: %d, max: %d, offset: %d", packetData->leftEngine.min, packetData->leftEngine.max, packetData->leftEngine.offset);
				ESP_LOGI("Transceiver", "Right engine min: %d, max: %d, offset: %d", packetData->rightEngine.min, packetData->rightEngine.max, packetData->rightEngine.offset);
				ESP_LOGI("Transceiver", "Left aileron min: %d, max: %d, offset: %d", packetData->leftAileron.min, packetData->leftAileron.max, packetData->leftAileron.offset);
				ESP_LOGI("Transceiver", "Right aileron min: %d, max: %d, offset: %d", packetData->rightAileron.min, packetData->rightAileron.max, packetData->rightAileron.offset);
				ESP_LOGI("Transceiver", "Left tail min: %d, max: %d, offset: %d", packetData->leftTail.min, packetData->leftTail.max, packetData->leftTail.offset);
				ESP_LOGI("Transceiver", "Right tail min: %d, max: %d, offset: %d", packetData->rightTail.min, packetData->rightTail.max, packetData->rightTail.offset);
				ESP_LOGI("Transceiver", "Left flap min: %d, max: %d, offset: %d", packetData->leftFlap.min, packetData->leftFlap.max, packetData->leftFlap.offset);
				ESP_LOGI("Transceiver", "Right flap min: %d, max: %d, offset: %d", packetData->rightFlap.min, packetData->rightFlap.max, packetData->rightFlap.offset);
				ESP_LOGI("Transceiver", "----------------");

				break;
			}
			case PacketDataType::RemoteLights: {
				if (!checkCRC(packetPtr, sizeof(RemoteLightsPacketData)))
					return;

				auto packetData = reinterpret_cast<RemoteLightsPacketData*>(packetPtr);

				rc.lights.setNavigationEnabled(packetData->navigation);
				rc.lights.setStrobeEnabled(packetData->strobe);
				rc.lights.setLandingEnabled(packetData->landing);

				ESP_LOGI("Transceiver", "-------- Lights --------");
				ESP_LOGI("Transceiver", "Nav lights: %d", rc.lights.isNavigationEnabled());
				ESP_LOGI("Transceiver", "Strobe lights: %d", rc.lights.isStrobeEnabled());
				ESP_LOGI("Transceiver", "Landing lights: %d", rc.lights.isLandingEnabled());
				ESP_LOGI("Transceiver", "----------------");

				break;
			}
			case PacketDataType::Aircraft: {
				break;
			}
		}
	}
}