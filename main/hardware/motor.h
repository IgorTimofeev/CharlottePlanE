#pragma once

#include <cmath>

namespace pizda {
	class Motor {
		public:
			Motor(
				const gpio_num_t pin,
				const ledc_channel_t channel,
				const uint16_t minPulseWidthUs = 1000,
				const uint16_t maxPulseWidthUs = 2000,
				const uint16_t frequencyHz = 50
			) :
				pin(pin),
				channel(channel),
				minPulseWidthUs(minPulseWidthUs),
				maxPulseWidthUs(maxPulseWidthUs),
				frequencyHz(frequencyHz)
			{

			}

			void setup(const uint16_t initialPulseWidth) const {
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
				channelConfig.duty = getDutyFromPulseWidth(initialPulseWidth);
				channelConfig.hpoint = 0;
				ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));
			}

			uint16_t getMinPulseWidth() const {
				return minPulseWidthUs;
			}

			void setMinPulseWidth(const uint16_t value) {
				this->minPulseWidthUs = value;
			}

			uint16_t getMaxPulseWidth() const {
				return maxPulseWidthUs;
			}

			void setMaxPulseWidth(const uint16_t value) {
				this->maxPulseWidthUs = value;
			}

			uint16_t getFrequency() const {
				return frequencyHz;
			}

			void setFrequency(const uint16_t value) {
				this->frequencyHz = value;
			}

			uint16_t getPulseWidthFromPercent(const uint8_t percent) const {
				return minPulseWidthUs + (maxPulseWidthUs - minPulseWidthUs) * percent / 100;
			}

			void setPulseWidth(const uint16_t pulseWidthUs) const {
				const auto duty = getDutyFromPulseWidth(pulseWidthUs);

//				ESP_LOGI("Motor", "setPulse() us: %f, duty: %f", (float) pulseWidthUs, (float) duty);

				ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty));
				ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel));
			}

			void setUint8(uint8_t value) const {
				setPulseWidth(minPulseWidthUs + (maxPulseWidthUs - minPulseWidthUs) * value / 0xFF);
			}

			void setUint16(uint16_t value) const {
				setPulseWidth(minPulseWidthUs + (maxPulseWidthUs - minPulseWidthUs) * value / 0xFFFF);
			}

			void setPercent(uint8_t value) const {
				setPulseWidth(getPulseWidthFromPercent(value));
			}

		private:
			gpio_num_t pin;
			ledc_channel_t channel;
			uint16_t minPulseWidthUs;
			uint16_t maxPulseWidthUs;
			uint16_t frequencyHz;

			constexpr static ledc_timer_bit_t _dutyResolution = LEDC_TIMER_13_BIT;

			uint32_t getDutyFromPulseWidth(const uint16_t pulseWidthUs) const {
				const auto dutyResolutionMaxValue = static_cast<uint16_t>(std::pow(2, static_cast<uint8_t>(_dutyResolution))) - 1;
				const auto duty = static_cast<uint32_t>(pulseWidthUs * dutyResolutionMaxValue / (1'000'000 / frequencyHz));

				return duty;
			}
	};
}