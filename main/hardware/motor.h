#pragma once

#include <cmath>
#include <algorithm>

#include <driver/gpio.h>
#include <driver/ledc.h>

namespace pizda {
	#pragma pack(push, 1)
	class MotorConfiguration {
		public:
			uint16_t min = 1000;
			uint16_t max = 2000;
			uint16_t startup = 1500;
			int16_t offset = 0;
			bool reverse = false;

			void sanitize() {
				min = std::clamp<uint16_t>(min, 100, 1400);
				max = std::clamp<uint16_t>(max, 1600, 2900);

				if (min > max)
					std::swap(min, max);

				startup = std::clamp<uint16_t>(startup, min, max);

				if (std::abs(offset) > 900)
					offset = 0;
			}
	};
	#pragma pack(pop)

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