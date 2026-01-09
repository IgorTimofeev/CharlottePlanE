#include "motor.h"

#include <cmath>
#include <algorithm>
#include <esp_log.h>

namespace pizda {
	Motor::Motor(const gpio_num_t pin, const ledc_channel_t channel) :
		_pin(pin),
		_channel(channel)
	{

	}

	void Motor::setup() {
		ledc_timer_config_t timerConfig {};
		timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
		timerConfig.duty_resolution = dutyBitCount;
		timerConfig.timer_num = LEDC_TIMER_0;
		timerConfig.freq_hz = frequencyHz;
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
	}

	void Motor::setDuty(uint32_t duty) const {
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, _channel, duty));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, _channel));
	}

	void Motor::setPulseWidth(uint16_t pulseWidth) const {
		// Pulse width -> duty cycle conversion
		const auto duty = static_cast<uint32_t>(pulseWidth * dutyMaxValue * frequencyHz / 1'000'000);

		setDuty(duty);
	}

	ConfiguredMotor::ConfiguredMotor(gpio_num_t pin, ledc_channel_t channel) : _motor(Motor(pin, channel)) {

	}

	void ConfiguredMotor::setup(const MotorConfiguration& configuration) {
		_motor.setup();
		
		_configuration = configuration;
		setStartupPower();
	}

	uint16_t ConfiguredMotor::getPower() const {
		return _power;
	}
	
	float ConfiguredMotor::getPowerF() {
		return static_cast<float>(getPower()) / static_cast<float>(Motor::powerMaxValue);
	}
	
	// Value range is [0; 0xFFFF]
	void ConfiguredMotor::setPower(uint16_t value) {
		_power = value;

		auto pulseWidth =
			_configuration.min
			+ (_configuration.max - _configuration.min) * _power / Motor::powerMaxValue
			+ _configuration.offset;

		if (_configuration.reverse)
			pulseWidth = _configuration.min + _configuration.max - pulseWidth;

		pulseWidth = std::clamp<int32_t>(pulseWidth, _configuration.min, _configuration.max);

		_motor.setPulseWidth(pulseWidth);
	}
	
	void ConfiguredMotor::setPowerF(float value) {
		setPower(value * Motor::powerMaxValue);
	}

	void ConfiguredMotor::updateCurrentPowerFromConfiguration() {
		setPower(_power);
	}

	void ConfiguredMotor::setStartupPower() {
		setPower((_configuration.startup - _configuration.min) * Motor::powerMaxValue / (_configuration.max - _configuration.min));
	}

	void ConfiguredMotor::setConfiguration(const MotorConfiguration& configuration) {
		_configuration = configuration;
	}
}