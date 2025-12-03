#pragma once

#include <cmath>

#include <driver/gpio.h>
#include <driver/ledc.h>

namespace pizda {
	class MotorSettings {
		public:
			uint16_t min = 0;
			uint16_t max = 0;
			int16_t offset = 0;
	};

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