#pragma once

#include <cstdint>

#include <driver/uart.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

namespace pizda {
	class constants {
		public:
			class common {
				public:
					constexpr static gpio_num_t reset = GPIO_NUM_NC;
			};

			class spi {
				public:
					constexpr static gpio_num_t miso = GPIO_NUM_13;
					constexpr static gpio_num_t mosi = GPIO_NUM_11;
					constexpr static gpio_num_t sck = GPIO_NUM_12;
					constexpr static spi_host_device_t device = SPI2_HOST;
			};

			class i2c {
				public:
					constexpr static gpio_num_t scl = GPIO_NUM_9;
					constexpr static gpio_num_t sda = GPIO_NUM_8;
			};

			class motors {
				public:
					constexpr static gpio_num_t throttle = GPIO_NUM_1;
					constexpr static gpio_num_t noseWheelSteering = GPIO_NUM_NC;

					constexpr static gpio_num_t leftWingFlap = GPIO_NUM_6;
					constexpr static gpio_num_t leftWingAileron = GPIO_NUM_5;

					constexpr static gpio_num_t rightWingFlap = GPIO_NUM_NC;
					constexpr static gpio_num_t rightWingAileron = GPIO_NUM_NC;

					constexpr static gpio_num_t tailLeft = GPIO_NUM_NC;
					constexpr static gpio_num_t tailRight = GPIO_NUM_NC;
			};

			class lights {
				public:
					class cabin {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_48;
							constexpr static uint8_t length = 1;
					};

					class leftWing {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_4;
							constexpr static uint8_t length = 6;
					};

					class rightWing {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_4;
							constexpr static uint8_t length = 6;
					};

					class tail {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_4;
							constexpr static uint8_t length = 3;
					};
			};

			class adiru1 {
				public:
					constexpr static uint8_t mpu9250Address = 0x68;
					constexpr static uint8_t bmp280Address = 0x76;

//					constexpr static gpio_num_t mpu9250ss = GPIO_NUM_17;
//					constexpr static gpio_num_t bmp280ss = GPIO_NUM_18;
			};

			class adiru2 {
				public:
//					constexpr static gpio_num_t mpu9250ss = GPIO_NUM_NC;
//					constexpr static gpio_num_t bmp280ss = GPIO_NUM_NC;
			};

			class transceiver {
				public:
					constexpr static gpio_num_t ss = GPIO_NUM_NC;
					constexpr static gpio_num_t busy = GPIO_NUM_NC;
					constexpr static gpio_num_t dio0 = GPIO_NUM_NC;
			};

			class gnss {
				public:
					constexpr static gpio_num_t rx = GPIO_NUM_NC;
					constexpr static gpio_num_t tx = GPIO_NUM_NC;
			};
	};
}
