#pragma once

#include <cstring>
#include <array>
#include <cmath>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_log.h>
#include <esp_log.h>
#include <esp_timer.h>

#include "hardware/ADIRS/ADIRS.h"
#include "hardware/simLink/simLink.h"

#include "types.h"
#include "config.h"

namespace pizda {
	class SimADIRS : public ADIRS {
		public:
			void setup() {
				start();
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
			
		protected:
			void onTick() override;
			void onCalibrateAccelAndGyro() override;
			void onCalibrateMag() override;
			
		private:
			void simulateCalibration();
	};
}