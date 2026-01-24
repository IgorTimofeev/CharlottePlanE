#include "ADIRS.h"

#include "aircraft.h"

namespace YOBA {
	void ADIRS::onStart() {
		auto& ac = Aircraft::getInstance();
		
		while (true) {
			if (
				ac.aircraftData.calibration.calibrating
				&& (
					ac.aircraftData.calibration.system == AircraftCalibrationSystem::accelAndGyro
					|| ac.aircraftData.calibration.system == AircraftCalibrationSystem::mag
				)
			) {
				if (ac.aircraftData.calibration.system == AircraftCalibrationSystem::accelAndGyro) {
					onCalibrateAccelAndGyro();
				}
				else {
					onCalibrateMag();
				}
				
				ac.aircraftData.calibration.calibrating = false;
			}
			else {
				onTick();
			}
		}
	}
}