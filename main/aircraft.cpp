#include "aircraft.h"

#include <string>
#include <iostream>

#include "esp_log.h"
#include "constants.h"
#include "esp_console.h"

#include <YOBABitStream/main.h>

namespace pizda {
	Aircraft& Aircraft::getInstance() {
		static auto instance = Aircraft();

		return instance;
	}

	void Aircraft::start() {
		settings.setup();

		SPIBusSetup();

		motors.setup();

		lights.setup();
		lights.start();

		transceiverSetup();
		channels.setup();

		while (true) {
//			ESP_LOGI("Main", "Pizda");

			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	void Aircraft::SPIBusSetup() const {
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

	void Aircraft::transceiverSetup() {
		transceiver.setup();
		transceiver.setPacketParser(&packetParser);
		transceiver.start();
	}

	void Aircraft::updateHardwareFromChannels() {
		// Motors
		{
			const auto getUintChannelAndUpdateMotor = [this](uint8_t channelIndex, uint8_t motorIndex) {
				const auto uintChannel = channels.getUintChannel(channelIndex, Motor::powerBitCount);

				if (!uintChannel)
					return;

				const auto motor = Aircraft::getInstance().motors.getMotor(motorIndex);

				if (!motor)
					return;

				motor->setPower(uintChannel->getValue());
			};

			getUintChannelAndUpdateMotor(2, 2);
			getUintChannelAndUpdateMotor(5, 6);
		}

		// Lights
		{
			BoolChannel* boolChannel;

			if ((boolChannel = channels.getBoolChannel(6)))
				lights.setNavigationEnabled(boolChannel->getValue());

			if ((boolChannel = channels.getBoolChannel(7)))
				lights.setStrobeEnabled(boolChannel->getValue());

			if ((boolChannel = channels.getBoolChannel(8)))
				lights.setLandingEnabled(boolChannel->getValue());

			if ((boolChannel = channels.getBoolChannel(9)))
				lights.setCabinEnabled(boolChannel->getValue());
		}
	}
}
