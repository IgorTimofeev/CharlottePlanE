#include "motor.h"

#include "aircraft.h"

namespace YOBA {
	Motor::Motor(PulseWidthModulator* PWM) : _PWM(PWM) {
	
	}
	
	uint16_t Motor::getPower() const {
		return _power;
	}
	
	float Motor::getPowerF() const {
		return static_cast<float>(getPower()) / static_cast<float>(powerMax);
	}
	
	// Value range is [0; 0xFFFF]
	void Motor::setPower(const uint16_t value) {
		_power = value;
		
		auto pulseWidthUs = _configuration.min + (_configuration.max - _configuration.min) * _power / powerMax;
		
		if (_configuration.reverse)
			pulseWidthUs = _configuration.min + _configuration.max - pulseWidthUs;
		
		pulseWidthUs = std::clamp<int32_t>(pulseWidthUs, _configuration.min, _configuration.max);

//		ESP_LOGI("ppizda", "pulse: %d", pulseWidthUs);

		_PWM->setPulseWidth(pulseWidthUs);
	}
	
	void Motor::setPowerF(const float value) {
		setPower(value * powerMax);
	}

	void Motor::setConfiguration(const MotorConfiguration& configuration) {
		_configuration = configuration;
	}
}