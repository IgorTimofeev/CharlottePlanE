#include "motors.h"

#include "rc.h"

namespace pizda {
	void Motors::setup() {
		leftAileronMotor.setup();
		setLeftAileron(0xFFFF / 2);

		leftFlapMotor.setup();
		setLeftFlap(0xFFFF / 2);
	}

	uint16_t Motors::getLeftAileron() const {
		return leftAileron;
	}

	void Motors::setLeftAileron(uint16_t value) {
		leftAileron = value;
		leftAileronMotor.setUint16(RC::getInstance().settings.controlsCalibration.leftAileron, leftAileron);
	}

	uint16_t Motors::getLeftFlap() const {
		return leftFlap;
	}

	void Motors::setLeftFlap(uint16_t value) {
		leftFlap = value;
		leftFlapMotor.setUint16(RC::getInstance().settings.controlsCalibration.leftFlap, leftFlap);
	}
}