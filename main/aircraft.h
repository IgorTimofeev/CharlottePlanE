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

namespace pizda {
	class Aircraft {
		public:
			Settings settings {};

			Lights lights {};
			Motors motors {};
			Channels channels {};

			Transceiver transceiver {};
			AircraftPacketParser packetParser {};

			static Aircraft& getInstance();

			void start();

		private:
			Aircraft() = default;

			void SPIBusSetup() const;
			void transceiverSetup();
	};
}
