#pragma once

#include <cstdint>

#include "esp_log.h"
#include "esp_timer.h"

#include "constants.h"
#include "settings.h"
#include "hardware/lights.h"
#include "hardware/motors.h"
#include "hardware/transceiver/transceiver.h"
#include "hardware/transceiver/RCPacketParser.h"

namespace pizda {
	class RC {
		public:
			Settings settings {};

			Lights lights {};
			Motors motors {};

			Transceiver transceiver {};
			RCPacketParser packetParser {};

			static RC& getInstance();

			void start();

		private:
			RC() = default;

			void SPIBusSetup() const;

			void transceiverSetup();
	};
}
