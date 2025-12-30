#include "aircraft.h"

#include <string>
#include <iostream>

#include "esp_log.h"
#include "config.h"
#include "esp_console.h"

#include <bitStream.h>

#include "utils/lowPassFilter.h"

namespace pizda {
	Aircraft& Aircraft::getInstance() {
		static auto instance = Aircraft();

		return instance;
	}

	void Aircraft::start() {
		settings.setup();

		SPIBusSetup();
		
		adirs.setup();
		
		motors.setup();

		lights.setup();
		lights.start();

		channels.setup();
		
		// Transceiver
		if (!transceiver.setup())
			startErrorLoop("failed to setup XCVR");
		
		packetHandler.setTransceiver(&transceiver);
		packetHandler.start();
		
		// Autopilot
		fbw.setup();
		fbw.start();
		
		while (true) {
//			ESP_LOGI("Main", "Pizda");
			
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}
	
	void Aircraft::startErrorLoop(const char* error) {
		ESP_LOGE(_logTag, "%s", error);
		
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(1'000));
		}
	}

	void Aircraft::SPIBusSetup() const {
		spi_bus_config_t busConfig {};
		busConfig.mosi_io_num = config::spi::MOSI;
		busConfig.miso_io_num = config::spi::MISO;
		busConfig.sclk_io_num = config::spi::SCK;
		busConfig.quadwp_io_num = -1;
		busConfig.quadhd_io_num = -1;
		busConfig.max_transfer_sz = 320 * 240;

		ESP_ERROR_CHECK(spi_bus_initialize(config::spi::device, &busConfig, SPI_DMA_CH_AUTO));
	}
}
