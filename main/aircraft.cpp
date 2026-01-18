#include "aircraft.h"

#include <string>
#include <iostream>

#include "esp_log.h"
#include "config.h"

#include <bitStream.h>

#include "utilities/lowPassFilter.h"

namespace pizda {
	Aircraft& Aircraft::getInstance() {
		static auto instance = Aircraft();

		return instance;
	}
	
	[[noreturn]] void Aircraft::start() {
		settings.setup();

		SPIBusSetup();
		
		adirs.setup();
		
		motors.setup();

		lights.setup();
		lights.start();

		// Transceiver
		if (!transceiver.setup())
			startErrorLoop("failed to setup XCVR");
		
		communicationManager.setTransceiver(&transceiver);
		communicationManager.start();
		
		// Autopilot
		fbw.setup();
		
		#ifdef SIM
			simLink.setup();
		#endif
		
		while (true) {
//			ESP_LOGI("Main", "Pizda");
			
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}
	
	[[noreturn]] void Aircraft::startErrorLoop(const char* error) {
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
