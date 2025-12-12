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

		ahrs.setup();

		while (true) {
//			ESP_LOGI("Main", "Pizda");

			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	void Aircraft::SPIBusSetup() const {
		spi_bus_config_t busConfig {};
		busConfig.mosi_io_num = constants::spi::mosi;
		busConfig.miso_io_num = constants::spi::miso;
		busConfig.sclk_io_num = constants::spi::sck;
		busConfig.quadwp_io_num = -1;
		busConfig.quadhd_io_num = -1;
		busConfig.max_transfer_sz = 320 * 240;

		ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_CH_AUTO));
	}

	void Aircraft::transceiverSetup() {
		transceiver.setup();
		transceiver.setPacketParser(&packetParser);
		transceiver.start();
	}

	void Aircraft::updateHardwareFromChannels() {
		// Throttle
		{
			const auto channel = channels.getUintChannel(ChannelType::throttle);
			const auto motor = motors.getMotor(MotorType::throttle);

			if (!channel || !motor)
				return;

			motor->setPower(channel->getValue());
		}

		// Ailerons
		{
			const auto channel = channels.getUintChannel(ChannelType::ailerons);
			const auto leftAileronMotor = motors.getMotor(MotorType::leftAileron);
//				const auto rightAileronMotor = motors.getMotor(MotorType::rightAileron);

			if (!channel || !leftAileronMotor)
				return;

			leftAileronMotor->setPower(channel->getValue());
//				rightAileronMotor->setPower(aileronsChannel->getValue());
		}

		// Flaps
		{
			const auto flapsChannel = channels.getUintChannel(ChannelType::flaps);
			const auto leftFlapMotor = motors.getMotor(MotorType::leftFlap);
//				const auto rightFlapMotor = motors.getMotor(MotorType::rightAileron);

			if (!flapsChannel || !leftFlapMotor)
				return;

			leftFlapMotor->setPower(flapsChannel->getValue());
//				rightFlapMotor->setPower(aileronsChannel->getValue());
		}

		// Lights
		{
			BoolChannel* boolChannel;

			if ((boolChannel = channels.getBoolChannel(ChannelType::navLights)))
				lights.setNavigationEnabled(boolChannel->getValue());

			if ((boolChannel = channels.getBoolChannel(ChannelType::strobeLights)))
				lights.setStrobeEnabled(boolChannel->getValue());

			if ((boolChannel = channels.getBoolChannel(ChannelType::landingLights)))
				lights.setLandingEnabled(boolChannel->getValue());
		}
	}
}
