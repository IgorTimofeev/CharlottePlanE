#pragma once

#include <cstdint>

#include <driver/uart.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c_master.h>

#include "utils/math.h"

namespace pizda {
	class config {
		public:
			class common {
				public:
					constexpr static gpio_num_t RST = GPIO_NUM_4;
			};

			class spi {
				public:
					constexpr static spi_host_device_t device = SPI2_HOST;
					
					constexpr static gpio_num_t MISO = GPIO_NUM_13;
					constexpr static gpio_num_t MOSI = GPIO_NUM_11;
					constexpr static gpio_num_t SCK = GPIO_NUM_12;
			};

			class i2c {
				public:
					constexpr static gpio_num_t SCL = GPIO_NUM_9;
					constexpr static gpio_num_t SDA = GPIO_NUM_8;
			};

			class motors {
				public:
					constexpr static gpio_num_t throttle = GPIO_NUM_1;

					constexpr static gpio_num_t leftFlap = GPIO_NUM_2;
					constexpr static gpio_num_t leftAileron = GPIO_NUM_42;

					constexpr static gpio_num_t rightFlap = GPIO_NUM_40;
					constexpr static gpio_num_t rightAileron = GPIO_NUM_39;

					constexpr static gpio_num_t tailLeft = GPIO_NUM_37;
					constexpr static gpio_num_t tailRight = GPIO_NUM_36;
					
					constexpr static gpio_num_t noseWheel = GPIO_NUM_47;
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
							constexpr static gpio_num_t pin = GPIO_NUM_41;
							constexpr static uint8_t length = 6;
					};

					class rightWing {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_38;
							constexpr static uint8_t length = 6;
					};

					class tail {
						public:
							constexpr static gpio_num_t pin = GPIO_NUM_35;
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
					// SX1262 supports up to 16 MHz, but with long wires (10+ cm) there will be troubles, so
					constexpr static uint32_t SPIFrequencyHz = 4'000'000;
					
					constexpr static gpio_num_t SS = GPIO_NUM_5;
					constexpr static gpio_num_t RST = GPIO_NUM_4;
					constexpr static gpio_num_t busy = GPIO_NUM_6;
					constexpr static gpio_num_t DIO1 = GPIO_NUM_7;
					
					constexpr static uint16_t RFFrequencyMHz = 915;
					constexpr static float bandwidthKHz = 500;
					constexpr static uint8_t spreadingFactor = 7;
					constexpr static uint8_t codingRate = 5;
					constexpr static uint8_t syncWord = 0x34;
					constexpr static uint16_t powerDBm = 22;
					constexpr static uint16_t preambleLength = 8;
			};

			class gnss {
				public:
					constexpr static gpio_num_t rx = GPIO_NUM_NC;
					constexpr static gpio_num_t tx = GPIO_NUM_NC;
			};
			
			class flyByWire {
				public:
					constexpr static float pitchAngleMaxRad = toRadians(15);
					constexpr static float rollAngleMaxRad = toRadians(30);
					constexpr static float aileronMaxFactor = 0.8f;
					constexpr static float elevatorMaxFactor = 0.8f;
			};
	};
}
