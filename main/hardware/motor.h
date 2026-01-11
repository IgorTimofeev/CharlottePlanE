#pragma once

#include <cmath>
#include <algorithm>

#include <driver/gpio.h>
#include <driver/ledc.h>

#include "types/generic.h"

namespace pizda {
	class Motor {
		public:
			Motor(gpio_num_t pin, ledc_channel_t channel);

			constexpr static const char* _logTag = "Motor";
			
			constexpr static uint16_t powerMaxValue = 0xFFFF;
			
			constexpr static uint8_t frequencyHz = 50;
			
			constexpr static ledc_timer_bit_t dutyBitCount = LEDC_TIMER_13_BIT;
			constexpr static uint32_t dutyMaxValue = 8191;

			void setup();
			void setDuty(uint32_t duty) const;
			void setPulseWidth(uint16_t pulseWidth) const;

		private:
			gpio_num_t _pin;
			ledc_channel_t _channel;
	};

	class ConfiguredMotor {
		public:
			ConfiguredMotor(gpio_num_t pin, ledc_channel_t channel);

			void setup(const MotorConfiguration& configuration);
			
			uint16_t getPower() const;
			float getPowerF();
			
			void setPower(uint16_t value);
			void setPowerF(float value);
			
			void updateCurrentPowerFromConfiguration();
			void setStartupPower();

			void setConfiguration(const MotorConfiguration& configuration);

		private:
			MotorConfiguration _configuration {};
			Motor _motor;
			uint16_t _power = 0;
	};
}