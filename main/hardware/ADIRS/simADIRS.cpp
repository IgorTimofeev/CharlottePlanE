#include "simADIRS.h"

#include "aircraft.h"

namespace pizda {
	void SimADIRS::onTick() {
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
	
	void SimADIRS::onCalibrateAccelAndGyro() {
		simulateCalibration();
	}
	
	void SimADIRS::onCalibrateMag() {
		simulateCalibration();
	}
	
	void SimADIRS::simulateCalibration() {
		auto& ac = Aircraft::getInstance();
		
		constexpr static uint32_t durationMs = 50'000;
		constexpr static uint32_t percentDurationMs = durationMs / 100;
		
		for (uint8_t i = 0; i < 100; ++i) {
			ac.aircraftData.calibration.progress = static_cast<uint8_t>(static_cast<uint32_t>(i) * 0xFF / 100);
			
			vTaskDelay(pdMS_TO_TICKS(percentDurationMs));
		}
		
		ac.aircraftData.calibration.progress = 0xFF;
	}
}