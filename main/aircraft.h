#pragma once

#include <cstdint>

#include <esp_log.h>
#include <esp_timer.h>

#include "constants.h"
#include "settings/settings.h"
#include "hardware/lights.h"
#include "hardware/motors.h"
#include "hardware/transceiver/transceiver.h"
#include "hardware/transceiver/aircraftPacketParser.h"
#include "hardware/transceiver/channels.h"
#include "hardware/ADIRS/ADIRS.h"
#include "hardware/transceiver/SX1262.h"

namespace pizda {
	class Aircraft {
		public:
			Settings settings {};

			Lights lights {};
			Motors motors {};
			Channels channels {};

			Transceiver transceiver {};
			AircraftPacketParser packetParser {};

			ADIRS ahrs {};

			SX1262 sx1262 {};
			
			static Aircraft& getInstance();
			void start();
			void updateHardwareFromChannels();

		private:
			constexpr static const char* _logTag = "Aircraft";
						
			Aircraft() = default;

			void SPIBusSetup() const;
			void transceiverSetup();
	};
}
