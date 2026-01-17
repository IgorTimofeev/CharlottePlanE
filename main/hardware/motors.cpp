#include <soc/gpio_struct.h>
#include "motors.h"

#include "aircraft.h"

namespace pizda {
	Motor::Motor(const gpio_num_t pin) : _pin(pin) {
	
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
		
		_pulseWidthUs = static_cast<uint16_t>(pulseWidthUs);
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
		
		getMotor(MotorType::throttle)->setConfiguration(ac.settings.motors.throttle);
		getMotor(MotorType::noseWheel)->setConfiguration(ac.settings.motors.noseWheel);
		getMotor(MotorType::aileronLeft)->setConfiguration(ac.settings.motors.aileronLeft);
		getMotor(MotorType::aileronRight)->setConfiguration(ac.settings.motors.aileronRight);
		getMotor(MotorType::flapLeft)->setConfiguration(ac.settings.motors.flapRight);
		getMotor(MotorType::flapRight)->setConfiguration(ac.settings.motors.flapLeft);
		getMotor(MotorType::tailLeft)->setConfiguration(ac.settings.motors.tailLeft);
		getMotor(MotorType::tailRight)->setConfiguration(ac.settings.motors.tailRight);
		
		for (auto& motor : _motors) {
			motor.setStartupPower();
		}
		
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
		
		// Timer 1
		gptimer_config_t timerConfig {};
		timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;
		timerConfig.direction = GPTIMER_COUNT_UP;
		timerConfig.resolution_hz = 1'000'000;
		timerConfig.intr_priority = 3;
		timerConfig.flags.intr_shared = false;
		timerConfig.flags.allow_pd = false;
		timerConfig.flags.backup_before_sleep = false;
		
		ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &_timer1));
		
		gptimer_event_callbacks_t timerEventCallbacks {};
		timerEventCallbacks.on_alarm = timer1AlarmCallback;
		
		ESP_ERROR_CHECK(gptimer_register_event_callbacks(_timer1, &timerEventCallbacks, this));
		
		gptimer_alarm_config_t timerAlarmConfig {};
		timerAlarmConfig.alarm_count = 1'000'000 / motorMaxPulseWidthFrequencyHz;
		timerAlarmConfig.reload_count = 0;
		timerAlarmConfig.flags.auto_reload_on_alarm = true;
		ESP_ERROR_CHECK(gptimer_set_alarm_action(_timer1, &timerAlarmConfig));
		
		ESP_ERROR_CHECK(gptimer_enable(_timer1));
		ESP_ERROR_CHECK(gptimer_start(_timer1));
		
		// Timer 2
		timerConfig = {};
		timerConfig.clk_src = GPTIMER_CLK_SRC_DEFAULT;
		timerConfig.direction = GPTIMER_COUNT_UP;
		timerConfig.resolution_hz = 1'000'000;
		timerConfig.intr_priority = 3;
		timerConfig.flags.intr_shared = false;
		timerConfig.flags.allow_pd = false;
		timerConfig.flags.backup_before_sleep = false;
		ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &_timer2));
		
		timerEventCallbacks = {};
		timerEventCallbacks.on_alarm = timer2AlarmCallback;
		ESP_ERROR_CHECK(gptimer_register_event_callbacks(_timer2, &timerEventCallbacks, this));
		
		ESP_ERROR_CHECK(gptimer_enable(_timer2));
	}
	
	bool Motors::timer1AlarmCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* eventData, void* userCtx) {
		auto m = reinterpret_cast<Motors*>(userCtx);
		
		const auto time = esp_timer_get_time();
		
		for (auto& motor : m->_motors) {
			if (motor._pulseWidthUs > 0) {
				gpio_set_level(motor._pin, true);
				
				motor._pulseDisableTimeTicks = time + motor._pulseWidthUs;
			}
			else {
				gpio_set_level(motor._pin, false);
				
				motor._pulseDisableTimeTicks = 0;
			}
		}
		
		m->updateClosest();
		
		return false;
	}
	
	bool Motors::timer2AlarmCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* eventData, void* userCtx) {
		auto m = reinterpret_cast<Motors*>(userCtx);
		
		auto& motor = m->_motors[m->_closestIndex];
		
		gpio_set_level(motor._pin, 0);
		
		motor._pulseDisableTimeTicks = 0;
		
		m->updateClosest();
		
		return false;
	}
	
	void Motors::updateClosest() {
		_closestIndex = 0xFF;
		
		const auto time = esp_timer_get_time();
		auto minDeltaUs = std::numeric_limits<int64_t>::max();
		esp_rom_printf("PZIDA NMAX: %lld\n", minDeltaUs);
		
		for (uint8_t i = 0; i < _motors.size(); ++i) {
			auto& motor = _motors[i];
			
			if (motor._pulseDisableTimeTicks == 0)
				continue;
			
			auto delta = motor._pulseDisableTimeTicks - time;
			
			if (delta < 1) {
				delta = 1;
			}
			
			if (delta < minDeltaUs) {
				minDeltaUs = delta;
				_closestIndex = i;
			}
		}
		
		if (_closestIndex == 0xFF) {
			ESP_ERROR_CHECK(gptimer_stop(_timer2));
		}
		else {
			esp_rom_printf("min delta: %lld\n", minDeltaUs);
			
			gptimer_alarm_config_t timerAlarmConfig {};
			timerAlarmConfig.alarm_count = minDeltaUs;
			timerAlarmConfig.reload_count = 0;
			timerAlarmConfig.flags.auto_reload_on_alarm = true;
			ESP_ERROR_CHECK(gptimer_set_alarm_action(_timer2, &timerAlarmConfig));
			
			ESP_ERROR_CHECK(gptimer_start(_timer2));
		}
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