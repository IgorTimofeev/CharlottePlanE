#include <string>
#include <iostream>
#include "rc.h"

#include "esp_log.h"
#include "config.h"
#include "esp_console.h"

namespace pizda {
	RC& RC::getInstance() {
		static auto instance = RC();

		return instance;
	}

	void RC::run() {
		SPIBusSetup();

		motors.setup();

		lights.setup();
		lights.start();

		transceiver.setup();
		transceiver.start();

		while (true) {
			ESP_LOGI("Main", "pizda");

			vTaskDelay(pdMS_TO_TICKS(1000));

			motors.leftWingAileron.setUint8(transceiver.lastPacket.leftWingAileron);
			motors.leftWingFlap.setUint8(transceiver.lastPacket.flaps);

			// Lights
			lights.setNavigationEnabled(transceiver.lastPacket.navigationLights);
			lights.setStrobeEnabled(transceiver.lastPacket.strobeLights);
			lights.setLandingEnabled(transceiver.lastPacket.landingLights);
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
}
