#pragma once

#include <cmath>

#include <driver/gpio.h>
#include <driver/ledc.h>
#include "settings.h"

namespace pizda {
	class Motor {
		public:
			Motor(gpio_num_t pin, ledc_channel_t channel);

			void setup();
			void setPower(const MotorSettings& settings, uint16_t power) const;
			void setPulseWidth(const MotorSettings& settings, uint16_t pulseWidth) const;

		private:
			gpio_num_t pin;
			ledc_channel_t channel;

			constexpr static uint8_t frequencyHz = 50;
			constexpr static ledc_timer_bit_t dutyResolution = LEDC_TIMER_13_BIT;

	};
}