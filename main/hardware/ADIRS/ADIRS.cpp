#include "ADIRS.h"

#include "aircraft.h"

namespace pizda {
	void ADIRS::onStart() {
		auto& ac = Aircraft::getInstance();
		
		while (true) {
			if (
				ac.aircraftData.state == AircraftState::calibration
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
				
				ac.aircraftData.state = AircraftState::normal;
			}
			else {
				onTick();
			}
		}
	}
}