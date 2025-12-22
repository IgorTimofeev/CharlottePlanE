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

//		motors.setup();
//
//		lights.setup();
//		lights.start();
//
//		transceiverSetup();
//		channels.setup();
//
//		ahrs.setup();
		
		sx1262.setup(
			constants::spi::device,
			constants::transceiver::ss,
			constants::transceiver::busy,
			constants::transceiver::dio1
		);
		
		
		sx1262.setBufferBaseAddress(0x00, 0x00);
		sx1262.setPacketType(SX1262::PACKET_TYPE_LORA);
		
		sx1262.setStandby(SX1262::STANDBY_RC);
		sx1262.setRxTxFallbackMode(SX1262::RX_TX_FALLBACK_MODE_STDBY_RC);

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

		ESP_ERROR_CHECK(spi_bus_initialize(constants::spi::device, &busConfig, SPI_DMA_CH_AUTO));
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
