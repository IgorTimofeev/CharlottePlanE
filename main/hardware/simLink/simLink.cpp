#include "simLink.h"

#include "aircraft.h"

namespace pizda {
	void SimLink::setup() {
		QueueHandle_t queue;
		ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, _bufferLength, _bufferLength, 10, &queue, 0));
		
		uart_config_t uartConfig {};
		uartConfig.baud_rate = 115200;
		uartConfig.data_bits = UART_DATA_8_BITS;
		uartConfig.parity = UART_PARITY_DISABLE;
		uartConfig.stop_bits = UART_STOP_BITS_1;
		uartConfig.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
		uartConfig.source_clk = UART_SCLK_DEFAULT;
		ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uartConfig));
		
		ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
		
		xTaskCreate(
			[](void* arg) {
				reinterpret_cast<SimLink*>(arg)->taskBody();
			},
			"SimLink",
			8 * 1024,
			this,
			10,
			nullptr
		);
	}
	
	void SimLink::taskBody() {
		auto& ac = Aircraft::getInstance();
		
		while (true) {
			const int bytesRead = uart_read_bytes(UART_NUM_0, _buffer, _bufferLength, pdMS_TO_TICKS(16));
			
			// Reading
			if (bytesRead >= sizeof(SimLinkSimPacket)) {
				auto packet = reinterpret_cast<SimLinkSimPacket*>(_buffer);
				
				if (packet->header == SimLinkPacket::header) {
					ac.adirs.fromSimPacket(*packet);
					
//					ESP_LOGI("SimLink", "pitchRad: %f",
//						toDegrees(packet->pitchRad)
//					);
				}
				else {
					ESP_LOGI("SimLink", "invalid header: %d", packet->header);
				}
			}
			
			// Writing
			{
//				ESP_LOGI("SimLink", "writing");
				
				auto packet = reinterpret_cast<SimLinkAircraftPacket*>(_buffer);
				
				const auto throttle = ac.motors.getMotor(MotorType::throttle);
				const auto ailerons = ac.motors.getMotor(MotorType::leftAileron);
				const auto elevator = ac.motors.getMotor(MotorType::leftTail);
				const auto rudder = ac.motors.getMotor(MotorType::rightTail);
				const auto flaps = ac.motors.getMotor(MotorType::leftFlap);
				
				packet->header = SimLinkPacket::header;
				packet->throttle = throttle ? throttle->getPowerF() : 0;
				packet->ailerons = ailerons ? ailerons->getPowerF() : 0;
				packet->elevator = elevator ? elevator->getPowerF() : 0;
				packet->rudder = rudder ? rudder->getPowerF() : 0;
				packet->flaps = flaps ? flaps->getPowerF() : 0;
				
				packet->lights =
					(ac.lights.isNavigationEnabled() << 0)
					| (ac.lights.isStrobeEnabled() << 1)
					| (ac.lights.isLandingEnabled() << 2)
					| (ac.lights.isCabinEnabled() << 3);
				
				uart_write_bytes(UART_NUM_0, _buffer, sizeof(SimLinkAircraftPacket));
			}
			
			vTaskDelay(pdMS_TO_TICKS(1'000 / 20));
		}
	}
}