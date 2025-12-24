#pragma once

#include <cstdint>

#include <esp_log.h>
#include <esp_timer.h>

#include "config.h"
#include "settings/settings.h"
#include "hardware/lights.h"
#include "hardware/motors.h"
#include "hardware/transceiver/transceiver.h"
#include "hardware/transceiver/aircraftPacketParser.h"
#include "hardware/transceiver/channels.h"
#include "hardware/ADIRS/ADIRS.h"

namespace pizda {
	using namespace YOBA;

	class Aircraft {
		public:
			Settings settings {};

			Lights lights {};
			Motors motors {};
			Channels channels {};

			Transceiver transceiver {};
			AircraftPacketParser packetParser {};

			ADIRS ahrs {};
			
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
