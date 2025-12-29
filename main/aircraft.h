#pragma once

#include <cstdint>

#include <esp_log.h>
#include <esp_timer.h>

#include "config.h"
#include "autopilot.h"
#include "settings/settings.h"
#include "hardware/lights.h"
#include "hardware/motors.h"
#include "hardware/transceiver/SX1262Transceiver.h"
#include "hardware/transceiver/aircraftPacketHandler.h"
#include "hardware/transceiver/channels.h"
#include "hardware/ADIRS/ADIRS.h"
#include "utils/remoteData.h"
#include "utils/aircraftData.h"

namespace pizda {
	using namespace YOBA;

	class Aircraft {
		public:
			Settings settings {};

			Lights lights {};
			Motors motors {};
			Channels channels {};
			
			SX1262Transceiver transceiver {};
			AircraftPacketHandler packetHandler {};

			ADIRS ahrs {};
			Autopilot autopilot {};
			
			RemoteData remoteData {};
			AircraftData aircraftData {};
			
			static Aircraft& getInstance();
			void start();
			void updateHardwareFromChannels();

		private:
			constexpr static const char* _logTag = "Aircraft";
			
			Aircraft() = default;

			void SPIBusSetup() const;
			
			void startErrorLoop(const char* error);
	};
}
