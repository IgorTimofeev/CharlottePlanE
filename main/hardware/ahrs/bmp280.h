#pragma once

#include <cstdint>
#include <utility>

#include <driver/gpio.h>
#include <driver/spi_master.h>

namespace pizda {
	enum class BMP280Oversampling : uint8_t {
		none = 0x00,
		x1 = 0x01,
		x2 = 0x02,
		x4 = 0x03,
		x8 = 0x04,
		x16 = 0x05
	};

	enum class BMP280Mode : uint8_t {
		sleep = 0x00,
		forced = 0x01,
		normal = 0x03,
		softReset = 0xB6
	};

	enum class BMP280Filter : uint8_t {
		none = 0x00,
		x2 = 0x01,
		x4 = 0x02,
		x8 = 0x03,
		x16 = 0x04
	};

	enum class BMP280StandbyDuration : uint8_t {
		ms1 = 0x00,
		ms63 = 0x01,
		ms125 = 0x02,
		ms250 = 0x03,
		ms500 = 0x04,
		ms1000 = 0x05,
		ms2000 = 0x06,
		ms4000 = 0x07
	};

	enum class BMP280Register : uint8_t {
		digT1 = 0x88,
		digT2 = 0x8A,
		digT3 = 0x8C,

		digP1 = 0x8E,
		digP2 = 0x90,
		digP3 = 0x92,
		digP4 = 0x94,
		digP5 = 0x96,
		digP6 = 0x98,
		digP7 = 0x9A,
		digP8 = 0x9C,
		digP9 = 0x9E,

		chipID = 0xD0,
		version = 0xD1,
		softReset = 0xE0,
		calibration = 0xE1,
		status = 0xF3,
		control = 0xF4,
		config = 0xF5,
		pressureData = 0xF7,
		temperatureData = 0xFA
	};

	class BMP280 {
		public:
			bool setup(
				spi_host_device_t SPIDevice,
				gpio_num_t misoPin,
				gpio_num_t mosiPin,
				gpio_num_t sckPin,
				gpio_num_t ssPin,
				uint32_t frequencyHz = 1'000'000
			) {
				// GPIO
				gpio_config_t GPIOConfig {};
				GPIOConfig.pin_bit_mask = 1ULL << ssPin;
				GPIOConfig.mode = GPIO_MODE_OUTPUT;
				GPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
				GPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
				GPIOConfig.intr_type = GPIO_INTR_DISABLE;
				gpio_config(&GPIOConfig);

				// Setting SS to high just in case
				gpio_set_level(ssPin, true);

				// SPI bus
				spi_bus_config_t busConfig {};
				busConfig.mosi_io_num = mosiPin;
				busConfig.miso_io_num = misoPin;
				busConfig.sclk_io_num = sckPin;
				busConfig.quadwp_io_num = -1;
				busConfig.quadhd_io_num = -1;
				busConfig.max_transfer_sz = 4092;

				// May be already initialized
				const auto result = spi_bus_initialize(SPIDevice, &busConfig, SPI_DMA_CH_AUTO);
				assert(result == ESP_OK || result == ESP_ERR_INVALID_STATE);

				// SPI interface
				spi_device_interface_config_t interfaceConfig {};
				interfaceConfig.mode = 0;
				interfaceConfig.clock_speed_hz = frequencyHz;
				interfaceConfig.spics_io_num = static_cast<int>(ssPin);
				interfaceConfig.queue_size = 1;
				interfaceConfig.flags = 0;
				interfaceConfig.command_bits = 8;

				ESP_ERROR_CHECK(spi_bus_add_device(SPIDevice, &interfaceConfig, &_SPIDeviceHandle));

				// Reading factory-fused calibration offsets
				readCalibrationData();

				// Configuring sensor to power-on-reset state
				configure(
					BMP280Mode::sleep,
					BMP280Oversampling::none,
					BMP280Oversampling::none,
					BMP280Filter::none,
					BMP280StandbyDuration::ms125
				);

				// Checking for valid chip ID & proper SPI wiring
				// From datasheet: "Chip ID can be read as soon as the device finished the power-on-reset"
				// ESP_LOGI("BMP", "chip id: %d", readUint8(BMP280Register::chipID));
				if (readUint8(BMP280Register::chipID) != chipID)
					return false;

				return true;
			}

			void configure(
				BMP280Mode mode,
				BMP280Oversampling pressureOversampling,
				BMP280Oversampling temperatureOversampling,
				BMP280Filter filter,
				BMP280StandbyDuration standbyDuration
			) {
				// t_sb = standbyDuration
				// filter = filter
				// spi3w_en = 0
				writeToRegister(
					BMP280Register::config,
					static_cast<uint8_t>(
						(std::to_underlying(standbyDuration) << 5)
						| (std::to_underlying(filter) << 2)
						| 0
					)
				);

				// osrs_t = temperatureOversampling
				// osrs_p = pressureOversampling
				// mode = mode
				writeToRegister(
					BMP280Register::control,
					static_cast<uint8_t>(
						(std::to_underlying(temperatureOversampling) << 5)
						| (std::to_underlying(pressureOversampling) << 2)
						| std::to_underlying(mode)
					)
				);
			}

			void readCalibrationData() {
				_calibrationDigT1 = readUint16LE(BMP280Register::digT1);
				_calibrationDigT2 = readInt16LE(BMP280Register::digT2);
				_calibrationDigT3 = readInt16LE(BMP280Register::digT3);

				_calibrationDigP1 = readUint16LE(BMP280Register::digP1);
				_calibrationDigP2 = readInt16LE(BMP280Register::digP2);
				_calibrationDigP3 = readInt16LE(BMP280Register::digP3);
				_calibrationDigP4 = readInt16LE(BMP280Register::digP4);
				_calibrationDigP5 = readInt16LE(BMP280Register::digP5);
				_calibrationDigP6 = readInt16LE(BMP280Register::digP6);
				_calibrationDigP7 = readInt16LE(BMP280Register::digP7);
				_calibrationDigP8 = readInt16LE(BMP280Register::digP8);
				_calibrationDigP9 = readInt16LE(BMP280Register::digP9);
			}

			// These bitchy compensation formulas has been taken from datasheet
			// Don't see any reason to touch them))0
			// ...
			// What are those var var lmao
			void readPressureAndTemperature(float& pressure, float& temperature) {
				// Temperature should be read first for tFine
				{
					int32_t adc_T = readInt24BE(BMP280Register::temperatureData);
					// Seems like this shit expects only last 20 bits from 24
					adc_T >>= 4;

					float var1, var2, T;
					var1 = (((float) adc_T) / 16384.0f - ((float) _calibrationDigT1) / 1024.0f) *
						   ((float) _calibrationDigT2);
					var2 = ((((float) adc_T) / 131072.0f - ((float) _calibrationDigT1) / 8192.0f) *
							(((float) adc_T) / 131072.0f - ((float) _calibrationDigT1) / 8192.0f)) *
						   ((float) _calibrationDigT3);

					_tFine = (int32_t)(var1 + var2);
					T = (var1 + var2) / 5120.0f;

					temperature = T;
				}

				// Pressure
				{
					int32_t adc_P = readInt24BE(BMP280Register::pressureData);
					adc_P >>= 4;

					float var1, var2, p;
					var1 = ((float) _tFine / 2.0f) - 64000.0f;
					var2 = var1 * var1 * ((float) _calibrationDigP6) / 32768.0f;
					var2 = var2 + var1 * ((float) _calibrationDigP5) * 2.0f;
					var2 = (var2 / 4.0f) + (((float) _calibrationDigP4) * 65536.0f);
					var1 =
						(((float) _calibrationDigP3) * var1 * var1 / 524288.0f + ((float) _calibrationDigP2) * var1) /
						524288.0f;
					var1 = (1.0f + var1 / 32768.0f) * ((float) _calibrationDigP1);

					// avoid exception caused by division by zero
					if (var1 == 0.0f) {
						pressure = 0;
					}
					else {
						p = 1048576.0f - (float) adc_P;
						p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
						var1 = ((float) _calibrationDigP9) * p * p / 2147483648.0f;
						var2 = p * ((float) _calibrationDigP8) / 32768.0f;
						p = p + (var1 + var2 + ((float) _calibrationDigP7)) / 16.0f;

						pressure = p;
					}
				}
			}

		private:
			constexpr static uint8_t chipID = 0x58;

			// From datasheet:
			// The variable t_fine (signed 32 bit) carries a fine resolution temperature value over to the
			// pressure compensation formula and could be implemented as a global variable
			int32_t _tFine = -0xFFFF;

			// SPI
			spi_device_handle_t _SPIDeviceHandle {};

			// Calibration data
			uint16_t _calibrationDigT1 = 0;
			int16_t _calibrationDigT2 = 0;
			int16_t _calibrationDigT3 = 0;

			uint16_t _calibrationDigP1 = 0;
			int16_t _calibrationDigP2 = 0;
			int16_t _calibrationDigP3 = 0;
			int16_t _calibrationDigP4 = 0;
			int16_t _calibrationDigP5 = 0;
			int16_t _calibrationDigP6 = 0;
			int16_t _calibrationDigP7 = 0;
			int16_t _calibrationDigP8 = 0;
			int16_t _calibrationDigP9 = 0;

			void writeToRegister(BMP280Register reg, uint8_t value) {
				spi_transaction_t transaction {};
				transaction.length = 2 * 8;
				transaction.tx_data[0] = static_cast<uint8_t>(std::to_underlying(reg) & ~0x80);
				transaction.tx_data[1] = value;
				transaction.flags = SPI_TRANS_USE_TXDATA;

				ESP_ERROR_CHECK(spi_device_transmit(_SPIDeviceHandle, &transaction));
			}

			void readFromRegister(BMP280Register reg, uint8_t* buffer, uint32_t readSize) {
				spi_transaction_t transaction {};
				transaction.cmd = static_cast<uint8_t>(std::to_underlying(reg) | 0x80);
				transaction.length = (readSize) * 8;
				transaction.rx_buffer = buffer;

				ESP_ERROR_CHECK(spi_device_transmit(_SPIDeviceHandle, &transaction));
			}

			uint16_t readUint8(BMP280Register reg) {
				uint8_t result = 0;
				readFromRegister(reg, &result, 1);

				return result;
			}

			uint16_t readUint16LE(BMP280Register reg) {
				uint8_t buffer[2];
				readFromRegister(reg, buffer, 2);

				return (static_cast<uint16_t>(buffer[1]) << 8) | static_cast<uint16_t>(buffer[0]);
			}

			int16_t readInt16LE(BMP280Register reg) {
				return static_cast<int16_t>(readUint16LE(reg));
			}

			uint32_t readUint24BE(BMP280Register reg) {
				uint8_t buffer[3];
				readFromRegister(reg, buffer, 3);

				return (static_cast<uint32_t>(buffer[0]) << 16) | (static_cast<uint32_t>(buffer[1]) << 8) | static_cast<uint32_t>(buffer[2]);
			}

			int32_t readInt24BE(BMP280Register reg) {
				return static_cast<int32_t>(readUint24BE(reg));
			}
	};
}