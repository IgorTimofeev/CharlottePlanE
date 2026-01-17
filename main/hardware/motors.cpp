#include <soc/gpio_struct.h>
#include "motors.h"

#include "aircraft.h"

namespace pizda {
	Motor::Motor(const gpio_num_t pin, ledc_channel_t channel) : _pin(pin), _channel(channel) {
	
	}
	
	void Motor::setup(const MotorConfiguration& configuration) {
		ledc_timer_config_t timerConfig {};
		timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
		timerConfig.duty_resolution = static_cast<ledc_timer_bit_t>(dutyLengthBits);
		timerConfig.timer_num = LEDC_TIMER_0;
		timerConfig.freq_hz = tickFrequencyHz;
		timerConfig.clk_cfg = LEDC_AUTO_CLK;
		ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));
		
		ledc_channel_config_t channelConfig {};
		channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
		channelConfig.channel = _channel;
		channelConfig.timer_sel = LEDC_TIMER_0;
		channelConfig.intr_type = LEDC_INTR_DISABLE;
		channelConfig.gpio_num = _pin;
		channelConfig.duty = 0;
		channelConfig.hpoint = 0;
		ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));
		
		setConfiguration(configuration);
		setStartupPower();
	}
	
	void Motor::setDuty(uint32_t duty) const {
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, duty));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel));
	}
	
	void Motor::setPulseWidth(uint16_t pulseWidth) const {
		// Pulse width -> duty cycle conversion
		const auto duty = static_cast<uint32_t>(pulseWidth * dutyMax / tickDurationUs);
		
		setDuty(duty);
	}
	
	uint16_t Motor::getPower() const {
		return _power;
	}
	
	float Motor::getPowerF() {
		return static_cast<float>(getPower()) / static_cast<float>(Motor::powerMax);
	}
	
	// Value range is [0; 0xFFFF]
	void Motor::setPower(uint16_t value) {
		_power = value;
		
		auto pulseWidthUs =
			_configuration.min
			+ (_configuration.max - _configuration.min) * _power / Motor::powerMax
			+ _configuration.offset;
		
		if (_configuration.reverse)
			pulseWidthUs = _configuration.min + _configuration.max - pulseWidthUs;
		
		pulseWidthUs = std::clamp<int32_t>(pulseWidthUs, _configuration.min, _configuration.max);
		
//		ESP_LOGI("ppizda", "pulse: %d", pulseWidthUs);
		
		setPulseWidth(pulseWidthUs);
	}
	
	void Motor::setPowerF(float value) {
		setPower(value * Motor::powerMax);
	}
	
	void Motor::updateCurrentPowerFromConfiguration() {
		setPower(_power);
	}
	
	void Motor::setStartupPower() {
		setPower((_configuration.startup - _configuration.min) * Motor::powerMax / (_configuration.max - _configuration.min));
	}
	
	void Motor::setConfiguration(const MotorConfiguration& configuration) {
		_configuration = configuration;
	}
	
	void Motors::setup() {
		// Motors
		auto& ac = Aircraft::getInstance();
		
		getMotor(MotorType::throttle)->setup(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setup(ac.settings.motors.noseWheel);
		getMotor(MotorType::aileronLeft)->setup(ac.settings.motors.aileronLeft);
		getMotor(MotorType::aileronRight)->setup(ac.settings.motors.aileronRight);
		getMotor(MotorType::flapLeft)->setup(ac.settings.motors.flapRight);
		getMotor(MotorType::flapRight)->setup(ac.settings.motors.flapLeft);
		getMotor(MotorType::tailLeft)->setup(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setup(ac.settings.motors.tailRight);
	}
	
	Motor* Motors::getMotor(uint8_t index) {
		if (index >= _motors.size()) {
			ESP_LOGI(_logTag, "index %d >= motors count %d", index, _motors.size());
			return nullptr;
		}
		
		return &_motors[index];
	}
	
	Motor* Motors::getMotor(MotorType type) {
		return getMotor(std::to_underlying(type));
	}

	void Motors::updateConfigurationsFromSettings() {
		auto& ac = Aircraft::getInstance();
		
		getMotor(MotorType::throttle)->setConfiguration(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setConfiguration(ac.settings.motors.noseWheel);
		getMotor(MotorType::aileronLeft)->setConfiguration(ac.settings.motors.aileronLeft);
		getMotor(MotorType::aileronRight)->setConfiguration(ac.settings.motors.aileronRight);
		getMotor(MotorType::flapLeft)->setConfiguration(ac.settings.motors.flapRight);
		getMotor(MotorType::flapRight)->setConfiguration(ac.settings.motors.flapLeft);
		getMotor(MotorType::tailLeft)->setConfiguration(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setConfiguration(ac.settings.motors.tailRight);
	}
	
}