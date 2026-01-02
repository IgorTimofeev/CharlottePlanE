#pragma once

#include <cstring>
#include <array>
#include <cmath>

#include <esp_log.h>

#include "hardware/ADIRS/ADIRS.h"
#include "hardware/simLink/simLink.h"

#include "config.h"

namespace pizda {
	class SimADIRS : public ADIRS {
		public:
			void setup() {
			
			}
			
			void fromSimPacket(const SimLinkSimPacket& packet) {
				updateSlipAndSkidFactor(packet.accelerationX, 2);
				
				setRollRad(packet.rollRad);
				setPitchRad(packet.pitchRad);
				setYawRad(packet.yawRad);
				updateHeadingFromYaw();
				
				setLatitude(packet.latitudeRad);
				setLongitude(packet.longitudeRad);
				
				setAccelSpeedMPS(packet.speedMPS);
				
				setPressurePa(packet.pressurePA);
				setTemperatureC(packet.temperatureC);
				updateAltitudeFromPressureTemperatureAndReferenceValue();
			}
	};
}