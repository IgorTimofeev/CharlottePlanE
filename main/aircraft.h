#pragma once

#include <cstdint>

#include <esp_log.h>
#include <esp_timer.h>

#include "config.h"
#include "aircraftData.h"
#include "remoteData.h"
#include "flyByWire.h"
#include "settings/settings.h"
#include "hardware/lights.h"
#include "hardware/motors.h"
#include "hardware/transceiver/SX1262Transceiver.h"
#include "hardware/transceiver/aircraftPacketHandler.h"

#define SIM

#ifdef SIM
	#include "hardware/ADIRS/simADIRS.h"
	#include "hardware/simLink/simLink.h"
#else
	#include "hardware/ADIRS/I2CADIRS.h"
#endif

namespace pizda {
	using namespace YOBA;

	class Aircraft {
		public:
			Settings settings {};

			Lights lights {};
			Motors motors {};
			
			SX1262Transceiver transceiver {};
			AircraftPacketHandler packetHandler {};
			
			#ifdef SIM
				SimADIRS adirs {};
				SimLink simLink {};
			#else
				I2CADIRS adirs {};
			#endif
			
			FlyByWire fbw {};
			
			AircraftData aircraftData {};
			RemoteData remoteData {};
			
			static Aircraft& getInstance();
			
			[[noreturn]] void start();

		private:
			constexpr static const char* _logTag = "Aircraft";
			
			Aircraft() = default;

			void SPIBusSetup() const;
			
			[[noreturn]] void startErrorLoop(const char* error);
	};
}
