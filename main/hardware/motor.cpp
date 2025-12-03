#include "motor.h"

#include <cmath>

namespace pizda {
	Motor::Motor(const gpio_num_t pin, const ledc_channel_t channel, const uint16_t frequency) :
		pin(pin),
		channel(channel),
		frequencyHz(frequency)
	{

	}

	void Motor::setup() const {
		ledc_timer_config_t timerConfig {};
		timerConfig.speed_mode = LEDC_LOW_SPEED_MODE;
		timerConfig.duty_resolution = _dutyResolution;
		timerConfig.timer_num = LEDC_TIMER_0;
		timerConfig.freq_hz = frequencyHz;
		timerConfig.clk_cfg = LEDC_AUTO_CLK;
		ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

		ledc_channel_config_t channelConfig {};
		channelConfig.speed_mode = LEDC_LOW_SPEED_MODE;
		channelConfig.channel = channel;
		channelConfig.timer_sel = LEDC_TIMER_0;
		channelConfig.intr_type = LEDC_INTR_DISABLE;
		channelConfig.gpio_num = pin;
		channelConfig.duty = 0;
		channelConfig.hpoint = 0;
		ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));
	}

	uint16_t Motor::getFrequency() const {
		return frequencyHz;
	}

	void Motor::setFrequency(const uint16_t value) {
		this->frequencyHz = value;
	}

	void Motor::setPulseWidth(const ControlsCalibrationSettingsMotor& settings, const uint16_t pulseWidth) const {
		const auto duty = getDutyFromPulseWidth(settings, pulseWidth);

		ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
	}

	void Motor::setUint16(const ControlsCalibrationSettingsMotor& settings, uint16_t value) const {
		setPulseWidth(settings, settings.min + (settings.max - settings.min) * value / 0xFFFF);
	}

	uint32_t Motor::getDutyFromPulseWidth(const ControlsCalibrationSettingsMotor& settings, const uint16_t pulseWidth) const {
		const auto clamped = static_cast<uint16_t>(std::clamp<int32_t>(static_cast<int32_t>(pulseWidth) + settings.offset, settings.min, settings.max));

		const auto dutyResolutionMaxValue = static_cast<uint16_t>(std::pow(2, static_cast<uint8_t>(_dutyResolution))) - 1;
		const auto duty = static_cast<uint32_t>(clamped * dutyResolutionMaxValue / (1'000'000 / frequencyHz));

		return duty;
	}
}