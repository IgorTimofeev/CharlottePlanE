#include <soc/gpio_struct.h>
#include "motors.h"

#include "aircraft.h"

namespace pizda {
	Motor::Motor(const gpio_num_t pin) : _pin(pin) {
	
	}
	
	void Motor::setup(const MotorConfiguration& configuration) {
		setConfiguration(configuration);
		setStartupPower();
	}
	
	
	uint16_t Motor::getPower() const {
		return _power;
	}
	
	float Motor::getPowerF() {
		return static_cast<float>(getPower()) / static_cast<float>(Motor::powerMax);
	}
	
	// Value range is [0; 0xFFFF]
	void Motor::setPower(uint16_t value) {
		_power = value;
		
		auto pulseWidthUs =
			_configuration.min
			+ (_configuration.max - _configuration.min) * _power / Motor::powerMax
			+ _configuration.offset;
		
		if (_configuration.reverse)
			pulseWidthUs = _configuration.min + _configuration.max - pulseWidthUs;
		
		pulseWidthUs = std::clamp<int32_t>(pulseWidthUs, _configuration.min, _configuration.max);
		
//		ESP_LOGI("ppizda", "pulse: %d", pulseWidthUs);

		_pulseWidthUs = pulseWidthUs;
	}
	
	void Motor::setPowerF(float value) {
		setPower(value * Motor::powerMax);
	}
	
	void Motor::updateCurrentPowerFromConfiguration() {
		setPower(_power);
	}
	
	void Motor::setStartupPower() {
		setPower((_configuration.startup - _configuration.min) * Motor::powerMax / (_configuration.max - _configuration.min));
	}
	
	void Motor::setConfiguration(const MotorConfiguration& configuration) {
		_configuration = configuration;
	}
	
	void Motors::setup() {
		// Motors
		auto& ac = Aircraft::getInstance();
		
		getMotor(MotorType::throttle)->setup(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setup(ac.settings.motors.noseWheel);
		getMotor(MotorType::aileronLeft)->setup(ac.settings.motors.aileronLeft);
		getMotor(MotorType::aileronRight)->setup(ac.settings.motors.aileronRight);
		getMotor(MotorType::flapLeft)->setup(ac.settings.motors.flapRight);
		getMotor(MotorType::flapRight)->setup(ac.settings.motors.flapLeft);
		getMotor(MotorType::tailLeft)->setup(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setup(ac.settings.motors.tailRight);
		
		// GPIO
		gpio_config_t gpioConfig {};
		gpioConfig.pin_bit_mask = 0;
		
		for (auto& motor : _motors)
			gpioConfig.pin_bit_mask |= (1ULL << static_cast<uint8_t>(motor._pin));
		
		gpioConfig.mode = GPIO_MODE_OUTPUT;
		gpioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
		gpioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
		gpioConfig.intr_type = GPIO_INTR_DISABLE;
		gpio_config(&gpioConfig);
		
		// Timer
		gptimer_config_t timerConfig {};
		timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;
		timerConfig.direction = GPTIMER_COUNT_UP;
		timerConfig.resolution_hz = 1'000'000;
		timerConfig.intr_priority = 3;
		timerConfig.flags.intr_shared = false;
		timerConfig.flags.allow_pd = false;
		timerConfig.flags.backup_before_sleep = false;
		
		ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &_timer));
		
		gptimer_event_callbacks_t timerEventCallbacks = {
			.on_alarm = timerAlarmCallback
		};
		
		ESP_ERROR_CHECK(gptimer_register_event_callbacks(_timer, &timerEventCallbacks, this));
		
		ESP_ERROR_CHECK(gptimer_enable(_timer));
		
		_startTime = esp_timer_get_time();
		updatePizda();
	}
	
	void Motors::updatePizda() {
//		esp_rom_printf("begin update\n");
		
		_closestIndex = 0xFF;
		
		const auto time = esp_timer_get_time();
		auto closestDelta = std::numeric_limits<int64_t>::max();
		
		_atLeastOneEnabled = false;
		
		for (uint8_t i = 0; i < _motors.size(); ++i) {
			auto& motor = _motors[i];
			
//			esp_rom_printf("motor %d\n", i);
			
			if (motor._disableTime == 0) {
//				esp_rom_printf("already disabled\n");
				
				continue;
			}
			
			auto delta = motor._disableTime - time;
			
//			esp_rom_printf("delta %lld closest %lld\n", delta, closestDelta);
			
			if (delta < 1)
				delta = 1;
			
			if (delta < closestDelta) {
//				esp_rom_printf("closest\n");
				
				_atLeastOneEnabled = true;
				
				closestDelta = delta;
				_closestIndex = i;
			}
		}
		
		gptimer_alarm_config_t timerAlarmConfig {};
		timerAlarmConfig.reload_count = 0;
		timerAlarmConfig.flags.auto_reload_on_alarm = true;
		
		if (_atLeastOneEnabled) {
//			esp_rom_printf("atLeastOneEnabled %d %lld\n", _closestIndex, closestDelta);
			
			timerAlarmConfig.alarm_count = closestDelta;
			
		}
		else {
			auto startDelta = time - _startTime;
			auto startDeltaMod = startDelta % pulsePeriodUs;
			auto startDeltaRemaining = pulsePeriodUs - startDeltaMod;
			
//			esp_rom_printf("all disabled %lld %lld %lld\n", startDelta, startDeltaMod, startDeltaRemaining);
			
			timerAlarmConfig.alarm_count = startDeltaRemaining;
		}
		
		ESP_ERROR_CHECK(gptimer_set_alarm_action(_timer, &timerAlarmConfig));
		ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_start(_timer));
	}
	
	void Motors::callbackInstance() {
		ESP_ERROR_CHECK_WITHOUT_ABORT(gptimer_stop(_timer));
		
		const auto time = esp_timer_get_time();
		
		uint32_t pinMask1 = 0;
		uint32_t pinMask2 = 0;
		
		if (_atLeastOneEnabled) {
			for (uint8_t i = 0; i < _motors.size(); ++i) {
				auto& motor = _motors[i];
				
				if (motor._disableTime <= time) {
//					esp_rom_printf("disabling %d\n", i);
					
					motor._disableTime = 0;
					
					const auto pin = static_cast<uint32_t>(motor._pin);
					
					if (pin < 32) {
						pinMask1 |= (1 << pin);
					}
					else {
						pinMask2 |= (1 << (pin - 32));
					}
				}
			}
			
			GPIO.out_w1tc = pinMask1;
			GPIO.out1_w1tc.val = pinMask2;
		}
		else {
//			esp_rom_printf("enabling all");
			
			for (auto& motor : _motors) {
				motor._disableTime = time + motor._pulseWidthUs;
				
				const auto pin = static_cast<uint32_t>(motor._pin);
				
				if (pin < 32) {
					pinMask1 |= (1 << pin);
				}
				else {
					pinMask2 |= (1 << (pin - 32));
				}
			}
			
			GPIO.out_w1ts = pinMask1;
			GPIO.out1_w1ts.val = pinMask2;
		}
		
		updatePizda();
	}
	
	bool Motors::timerAlarmCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* eventData, void* userCtx) {
		auto m = reinterpret_cast<Motors*>(userCtx);
		
		m->callbackInstance();
		
		return false;
	}
	
	Motor* Motors::getMotor(uint8_t index) {
		if (index >= _motors.size()) {
			ESP_LOGI(_logTag, "index %d >= motors count %d", index, _motors.size());
			return nullptr;
		}
		
		return &_motors[index];
	}
	
	Motor* Motors::getMotor(MotorType type) {
		return getMotor(std::to_underlying(type));
	}

	void Motors::updateConfigurationsFromSettings() {
		auto& ac = Aircraft::getInstance();
		
		getMotor(MotorType::throttle)->setConfiguration(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setConfiguration(ac.settings.motors.noseWheel);
		getMotor(MotorType::aileronLeft)->setConfiguration(ac.settings.motors.aileronLeft);
		getMotor(MotorType::aileronRight)->setConfiguration(ac.settings.motors.aileronRight);
		getMotor(MotorType::flapLeft)->setConfiguration(ac.settings.motors.flapRight);
		getMotor(MotorType::flapRight)->setConfiguration(ac.settings.motors.flapLeft);
		getMotor(MotorType::tailLeft)->setConfiguration(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setConfiguration(ac.settings.motors.tailRight);
	}
}