#include "rc.h"

#include <string>
#include <iostream>

#include "esp_log.h"
#include "constants.h"
#include "esp_console.h"

#include <YOBABitStream/main.h>

namespace pizda {
	RC& RC::getInstance() {
		static auto instance = RC();

		return instance;
	}

	void RC::start() {
		settings.setup();

		SPIBusSetup();

		motors.setup();

		lights.setup();
		lights.start();

		transceiverSetup();

		// Ailerons
		channels.bindings[2] = &motors.motors[2].value();
		// Flaps
		channels.bindings[5] = &motors.motors[6].value();
		// Lights
		channels.bindings[6] = &lights.navChannel;
		channels.bindings[7] = &lights.strobeChannel;
		channels.bindings[8] = &lights.landingChannel;
		channels.bindings[9] = &lights.cabinChannel;

		while (true) {
//			ESP_LOGI("Main", "Pizda");

			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	void RC::SPIBusSetup() const {
//		spi_bus_config_t busConfig {};
//		busConfig.mosi_io_num = config::spi::mosi;
//		busConfig.miso_io_num = config::spi::miso;
//		busConfig.sclk_io_num = config::spi::sck;
//		busConfig.quadwp_io_num = -1;
//		busConfig.quadhd_io_num = -1;
//		busConfig.max_transfer_sz = static_cast<int32_t>(display.getSize().getSquare()) * 2;
//
//		ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_CH_AUTO));
	}

	void RC::transceiverSetup() {
		transceiver.setup();
		transceiver.setPacketParser(&packetParser);
		transceiver.start();
	}
}
