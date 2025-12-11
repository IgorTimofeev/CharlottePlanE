#pragma once

#include <cstdint>

#include "esp_log.h"
#include "esp_timer.h"

#include "constants.h"
#include "settings/settings.h"
#include "hardware/lights.h"
#include "hardware/motors.h"
#include "hardware/transceiver/transceiver.h"
#include "hardware/transceiver/aircraftPacketParser.h"
#include "hardware/transceiver/channels.h"
#include "hardware/ahrs/ahrs.h"

namespace pizda {
	class Aircraft {
		public:
			Settings settings {};

			Lights lights {};
			Motors motors {};
			Channels channels {};

			Transceiver transceiver {};
			AircraftPacketParser packetParser {};

			AHRS ahrs {
				constants::adiru1::mpu9250ss,
				constants::adiru1::bmp280ss,
			};

			static Aircraft& getInstance();
			void start();
			void updateHardwareFromChannels();

		private:
			Aircraft() = default;

			void SPIBusSetup() const;
			void transceiverSetup();
	};
}
