#pragma once

#define SIM

#include <esp_adc/adc_oneshot.h>

#include <battery.h>

#include "config.h"
#include "settings/settings.h"
#include "systems/lights.h"
#include "systems/motors.h"
#include "systems/transceiver/SX1262Transceiver.h"
#include "systems/aircraftCommunicationManager.h"
#include "types/aircraftData.h"
#include "types/remoteData.h"
#include "systems/flyByWire.h"

#ifdef SIM
	#include "systems/ADIRS/simADIRS.h"
#include "systems/simLink/simLink.h"
#else
	#include "systems/ADIRS/I2CADIRS.h"
#endif

namespace YOBA {
	using namespace YOBA;

	class Aircraft {
		public:
			Settings settings {};

			Lights lights {};
			Motors motors {};

			Battery<
				config::battery::unit,
				config::battery::channel,

				config::battery::voltageMin,
				config::battery::voltageMax,
				config::battery::voltageDividerR1,
				config::battery::voltageDividerR2
			>
			battery { getAssignedADCOneshotUnit(config::battery::unit) };
			
			SX1262Transceiver transceiver {};
			AircraftCommunicationManager communicationManager {};
			
			#ifdef SIM
				SimLink simLink {};
				SimADIRS adirs {};
			#else
				I2CADIRS adirs {};
			#endif
			
			FlyByWire fbw {};
			
			AircraftData aircraftData {};
			RemoteData remoteData {};
			
			static Aircraft& getInstance();
			
			[[noreturn]] void start();

		private:
			constexpr static auto _logTag = "Aircraft";
			
			Aircraft() = default;

			adc_oneshot_unit_handle_t _ADCOneshotUnit2 {};

			static void SPIBusSetup();
			void ADCSetup();

			constexpr adc_oneshot_unit_handle_t* getAssignedADCOneshotUnit(const adc_unit_t ADCUnit) {
				switch (ADCUnit) {
					case ADC_UNIT_2: return &_ADCOneshotUnit2;
					default: startErrorLoop("failed to find assigned ADC oneshot unit");
				}
			}

			[[noreturn]] static void startErrorLoop(const char* error);
	};
}
