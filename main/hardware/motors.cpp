#include "motors.h"

#include "rc.h"

namespace pizda {
	void Motors::setup() {
		leftAileronMotor.setup();
		setAilerons(Motor::powerMaxValue / 2);

		leftFlapMotor.setup();
		setFlaps(Motor::powerMaxValue / 2);
	}

	uint16_t Motors::getThrottle() const {
		return throttle;
	}

	void Motors::setThrottle(uint16_t value) {
		throttle = value;
	}

	uint16_t Motors::getElevator() const {
		return elevator;
	}

	void Motors::setElevator(uint16_t value) {
		elevator = value;
	}

	uint16_t Motors::getRudder() const {
		return rudder;
	}

	void Motors::setRudder(uint16_t value) {
		rudder = value;
	}

	uint16_t Motors::getAilerons() const {
		return ailerons;
	}

	void Motors::setAilerons(uint16_t value) {
		ailerons = value;

		leftAileronMotor.setPower(RC::getInstance().settings.motors.leftAileron, ailerons);
	}

	uint16_t Motors::getFlaps() const {
		return flaps;
	}

	void Motors::setFlaps(uint16_t value) {
		flaps = value;

		leftFlapMotor.setPower(RC::getInstance().settings.motors.leftFlap, flaps);
	}
}