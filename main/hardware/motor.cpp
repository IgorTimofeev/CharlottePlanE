#include "motor.h"

#include <cmath>
#include <algorithm>
#include <esp_log.h>

namespace pizda {
	Motor::Motor(const gpio_num_t pin, const ledc_channel_t channel) :
		pin(pin),
		channel(channel)
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
		channelConfig.channel = channel;
		channelConfig.timer_sel = LEDC_TIMER_0;
		channelConfig.intr_type = LEDC_INTR_DISABLE;
		channelConfig.gpio_num = pin;
		channelConfig.duty = 0;
		channelConfig.hpoint = 0;
		ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));
	}

	void Motor::setDuty(uint32_t duty) const {
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
	}

	void Motor::setPulseWidth(uint16_t pulseWidth) const {
		// Pulse width -> duty cycle conversion
		const auto duty = static_cast<uint32_t>(pulseWidth * dutyMaxValue * frequencyHz / 1'000'000);

		setDuty(duty);
	}

	YobaMotor::YobaMotor(const Motor& motor) : motor(motor) {

	}

	void YobaMotor::setup() {
		motor.setup();
		setStartupPower();
	}

	uint16_t YobaMotor::getPower() const {
		return power;
	}

	void YobaMotor::setPower(uint16_t value) {
		power = value;

		const auto pulseWidth = configuration.min + (configuration.max - configuration.min) * power / Motor::powerMaxValue;

		auto pizda = static_cast<int32_t>(pulseWidth) + configuration.offset;

		if (configuration.reverse)
			pizda = configuration.min + configuration.max - pizda;

		pizda = std::clamp<int32_t>(pizda, configuration.min, configuration.max);

		motor.setPulseWidth(pizda);
	}

	void YobaMotor::setStartupPower() {
		setPower((configuration.startup - configuration.min) * Motor::powerMaxValue / (configuration.max - configuration.min));
	}

	void YobaMotor::onChannelValueChanged(uint32_t value) {
		setPower(value);
	}
}