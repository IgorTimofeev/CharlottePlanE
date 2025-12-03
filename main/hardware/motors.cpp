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
		leftAileronMotor.setPower(RC::getInstance().settings.motors.leftAileron, leftAileron);
	}

	uint16_t Motors::getLeftFlap() const {
		return leftFlap;
	}

	void Motors::setLeftFlap(uint16_t value) {
		leftFlap = value;
		leftFlapMotor.setPower(RC::getInstance().settings.motors.leftFlap, leftFlap);
	}
}