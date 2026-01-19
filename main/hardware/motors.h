#pragma once

#include <array>
#include <utility>
#include <atomic>

#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_timer.h>

#include "config.h"
#include "types/generic.h"

namespace pizda {
	class Motor {
			friend class Motors;
		
		public:
			Motor(gpio_num_t pin, ledc_channel_t channel);
			
			constexpr static const char* _logTag = "Motor";
			
			constexpr static uint16_t powerMax = 0xFFFF;
			
			constexpr static uint8_t tickFrequencyHz = 50;
			constexpr static uint32_t tickDurationUs = 1'000'000 / tickFrequencyHz;
			
			constexpr static uint8_t dutyLengthBits = 13;
			constexpr static uint32_t dutyMax = (1 << dutyLengthBits) - 1;
			
			void setup() const;
			void setPulseWidth(uint16_t pulseWidth) const;
			void setDuty(uint32_t duty) const;
			
			uint16_t getPower() const;
			float getPowerF() const;
			
			void setPower(uint16_t value);
			void setPowerF(float value);
			
			void updateCurrentPowerFromConfiguration();
			
			void setConfiguration(const MotorConfiguration& configuration);
		
		private:
			gpio_num_t _pin;
			ledc_channel_t _channel;
			
			MotorConfiguration _configuration {};
			
			uint16_t _power = 0;
		
	};
	
	enum class MotorType : uint8_t {
		throttle,
		noseWheel,
		
		flapLeft,
		aileronLeft,
		
		flapRight,
		aileronRight,
		
		tailLeft,
		tailRight
	};
	
	class Motors {
		public:
			void setup();
			Motor* getMotor(uint8_t index);
			Motor* getMotor(MotorType type);
			void updateConfigurationsFromSettings();
		
		private:
			constexpr static const char* _logTag = "Motors";
			
			std::array<Motor, 8> _motors {
				Motor { config::motors::throttle, LEDC_CHANNEL_0 },
				Motor { config::motors::noseWheel, LEDC_CHANNEL_1 },
				
				Motor { config::motors::flapLeft, LEDC_CHANNEL_2 },
				Motor { config::motors::aileronLeft, LEDC_CHANNEL_3 },
				
				Motor { config::motors::flapRight, LEDC_CHANNEL_4 },
				Motor { config::motors::aileronRight, LEDC_CHANNEL_5 },
				
				Motor { config::motors::tailLeft, LEDC_CHANNEL_6 },
				Motor { config::motors::tailRight, LEDC_CHANNEL_7 },
			};
	};
}