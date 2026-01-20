#include "aircraft.h"

#include <esp_log.h>

#include "config.h"

namespace pizda {
	Aircraft& Aircraft::getInstance() {
		static auto instance = Aircraft();

		return instance;
	}
	
	[[noreturn]] void Aircraft::start() {
		SPIBusSetup();
		ADCSetup();

		settings.setup();

		adirs.setup();
		motors.setup();
		battery.setup();

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

			battery.tick();
			
			vTaskDelay(pdMS_TO_TICKS(1'000 / 30));
		}
	}
	
	[[noreturn]] void Aircraft::startErrorLoop(const char* error) {
		ESP_LOGE(_logTag, "%s", error);
		
		while (true) {
			vTaskDelay(pdMS_TO_TICKS(1'000));
		}
	}

	void Aircraft::SPIBusSetup() {
		spi_bus_config_t busConfig {};
		busConfig.mosi_io_num = config::spi::MOSI;
		busConfig.miso_io_num = config::spi::MISO;
		busConfig.sclk_io_num = config::spi::SCK;
		busConfig.quadwp_io_num = -1;
		busConfig.quadhd_io_num = -1;
		busConfig.max_transfer_sz = 320 * 240;

		ESP_ERROR_CHECK(spi_bus_initialize(config::spi::device, &busConfig, SPI_DMA_CH_AUTO));
	}

	void Aircraft::ADCSetup() {
		adc_oneshot_unit_init_cfg_t unitConfig {};
		unitConfig.unit_id = ADC_UNIT_2;
		unitConfig.clk_src = ADC_RTC_CLK_SRC_DEFAULT;
		unitConfig.ulp_mode = ADC_ULP_MODE_DISABLE;
		ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitConfig, &_ADCOneshotUnit2));
	}
}
