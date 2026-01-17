#pragma once

#include <array>
#include <utility>
#include <atomic>

#include <driver/gpio.h>
#include <driver/gptimer.h>
#include <esp_timer.h>

#include "config.h"
#include "types/generic.h"

namespace pizda {
	class Motor {
		friend class Motors;

		public:
			Motor(gpio_num_t pin);
			
			constexpr static const char* _logTag = "Motor";
			
			constexpr static uint16_t powerMax = 0xFFFF;
			
			constexpr static uint8_t frequencyHz = 50;
			
			constexpr static uint8_t dutyLengthBits = 13;
			constexpr static uint32_t dutyMax = (1 << dutyLengthBits) - 1;
			
			uint16_t getPower() const;
			float getPowerF();
			
			void setPower(uint16_t value);
			void setPowerF(float value);
			
			void updateCurrentPowerFromConfiguration();
			void setStartupPower();
			
			void setConfiguration(const MotorConfiguration& configuration);
		
		private:
			gpio_num_t _pin;
			
			MotorConfiguration _configuration {};
			
			uint16_t _pulseWidthUs = 0;
			uint16_t _power = 0;
			
			int64_t _pulseDisableTimeTicks = 0;
	};
	
	enum class MotorType : uint8_t {
		throttle,
		noseWheel,

		aileronLeft,
		aileronRight,
		
		flapLeft,
		flapRight,
		
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
			
			constexpr static uint8_t motorMaxPulseWidthFrequencyHz = 50;
			constexpr static uint32_t motorMaxPulseWidthUs = 1'000'000 / motorMaxPulseWidthFrequencyHz;
			constexpr static uint32_t motorMinPulseWidthUs = motorMaxPulseWidthUs / 2;
			
			IRAM_ATTR static bool timer1AlarmCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* eventData, void* userCtx);
			IRAM_ATTR static bool timer2AlarmCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* eventData, void* userCtx);
			
			gptimer_handle_t _timer1;
			gptimer_handle_t _timer2;
			
			volatile uint8_t _closestIndex = 0xFF;
			
			void updateClosest();
			
			std::array<Motor, 8> _motors {
				Motor { config::motors::throttle },
				Motor { config::motors::noseWheel },

				Motor { config::motors::aileronLeft },
				Motor { config::motors::aileronRight },
				
				Motor { config::motors::flapLeft },
				Motor { config::motors::flapRight },
				
				Motor { config::motors::tailLeft },
				Motor { config::motors::tailRight },
			};
	};
}