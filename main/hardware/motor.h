#pragma once

#include <cmath>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include "settings/settings.h"
#include "channels.h"

namespace pizda {
	class Motor {
		public:
			Motor(gpio_num_t pin, ledc_channel_t channel);

			constexpr static uint8_t frequencyHz = 50;

			constexpr static uint8_t powerBitCount = 12;
			constexpr static uint16_t powerMaxValue = 4095;

			constexpr static ledc_timer_bit_t dutyBitCount = LEDC_TIMER_13_BIT;
			constexpr static uint32_t dutyMaxValue = 8191;

			void setup();
			void setDuty(uint32_t duty) const;
			void setPulseWidth(uint16_t pulseWidth) const;

		private:
			gpio_num_t pin;
			ledc_channel_t channel;

	};

	class YobaMotor : public ChannelAware {
		public:
			YobaMotor(const Motor& motor);

			Motor motor;
			MotorConfiguration configuration {};

			void setup();
			void setPower(uint16_t value);
			void setStartupPower();

			uint16_t getPower() const;

			void fromChannel(uint32_t value) override;

		private:
			uint16_t power = 0;
	};
}