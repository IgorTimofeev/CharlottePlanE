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
		
		gptimer_alarm_config_t timerAlarmConfig {};
		timerAlarmConfig.alarm_count = 1'000'000 / motorMaxPulseWidthFrequencyHz;
		timerAlarmConfig.reload_count = 0;
		timerAlarmConfig.flags.auto_reload_on_alarm = true;
		ESP_ERROR_CHECK(gptimer_set_alarm_action(_timer, &timerAlarmConfig));
		
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
		
		ESP_ERROR_CHECK(gptimer_enable(_timer));
		ESP_ERROR_CHECK(gptimer_start(_timer));
	}
	
	bool Motors::timerAlarmCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* eventData, void* userCtx) {
		auto m = reinterpret_cast<Motors*>(userCtx);
		
	//	if (m->_timerTick % 100'000 == 0)
	//	esp_rom_printf("pizda: %d", m->_timerTick);
		
		m->_timerTick = m->_timerTick + 1;
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