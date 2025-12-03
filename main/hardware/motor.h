#pragma once

#include <cmath>

#include <driver/gpio.h>
#include <driver/ledc.h>

#include "settings.h"

namespace pizda {
	class Motor {
		public:
			Motor(
				gpio_num_t pin,
				ledc_channel_t channel,
				uint16_t frequency = 50
			);

			void setup() const;

			uint16_t getFrequency() const;
			void setFrequency(uint16_t value);

			void setPulseWidth(const ControlsCalibrationSettingsMotor& settings, uint16_t pulseWidth) const;
			void setUint16(const ControlsCalibrationSettingsMotor& settings, uint16_t value) const;

		private:
			gpio_num_t pin;
			ledc_channel_t channel;
			uint16_t frequencyHz;

			constexpr static ledc_timer_bit_t _dutyResolution = LEDC_TIMER_13_BIT;

			uint32_t getDutyFromPulseWidth(const ControlsCalibrationSettingsMotor& settings, uint16_t pulseWidth) const;
	};
}