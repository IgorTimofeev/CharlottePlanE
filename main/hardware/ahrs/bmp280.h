#pragma once

#include <cstdint>

#include "driver/gpio.h"
#include "driver/spi_master.h"

namespace pizda {
	enum class BMP280Oversampling : uint8_t {
		None = 0x00,
		X1 = 0x01,
		X2 = 0x02,
		X4 = 0x03,
		X8 = 0x04,
		X16 = 0x05
	};

	enum class BMP280Mode : uint8_t {
		Sleep = 0x00,
		Forced = 0x01,
		Normal = 0x03,
		SoftReset = 0xB6
	};

	enum class BMP280Filter : uint8_t {
		None = 0x00,
		X2 = 0x01,
		X4 = 0x02,
		X8 = 0x03,
		X16 = 0x04
	};

	enum class BMP280StandbyDuration : uint8_t {
		Ms1 = 0x00,
		Ms63 = 0x01,
		Ms125 = 0x02,
		Ms250 = 0x03,
		Ms500 = 0x04,
		Ms1000 = 0x05,
		Ms2000 = 0x06,
		Ms4000 = 0x07
	};

	enum class BMP280Register : uint8_t {
		DigT1 = 0x88,
		DigT2 = 0x8A,
		DigT3 = 0x8C,

		DigP1 = 0x8E,
		DigP2 = 0x90,
		DigP3 = 0x92,
		DigP4 = 0x94,
		DigP5 = 0x96,
		DigP6 = 0x98,
		DigP7 = 0x9A,
		DigP8 = 0x9C,
		DigP9 = 0x9E,

		ChipID = 0xD0,
		Version = 0xD1,
		SoftReset = 0xE0,
		Calibration = 0xE1,
		Status = 0xF3,
		Control = 0xF4,
		Config = 0xF5,
		PressureData = 0xF7,
		TemperatureData = 0xFA
	};

	class BMP280 {
		public:
			BMP280(gpio_num_t misoPin, gpio_num_t mosiPin, gpio_num_t sckPin, gpio_num_t ssPin) : _misoPin(misoPin), _mosiPin(mosiPin), _sckPin(sckPin), _ssPin(ssPin) {

			}

			bool setup() {
				setupGPIO();
				setSlaveSelect(true);

				setupSPI();

				// Reading factory-fused calibration offsets
				readCalibrationData();

				// Configuring sensor to power-on-reset state
				configure(
					BMP280Mode::Sleep,
					BMP280Oversampling::None,
					BMP280Oversampling::None,
					BMP280Filter::None,
					BMP280StandbyDuration::Ms1
				);

				// Checking for valid chip ID & proper SPI wiring
				// From datasheet: "Chip ID can be read as soon as the device finished the power-on-reset"
				const auto chipID = readUint8(BMP280Register::ChipID);

				ESP_LOGI("BMP", "chip id: %d", chipID);

				if (readUint8(BMP280Register::ChipID) != 0x58)
					return false;

				return true;
			}

			void configure(
				BMP280Mode mode = BMP280Mode::Normal,
				BMP280Oversampling temperatureOversampling = BMP280Oversampling::X2,
				BMP280Oversampling pressureOversampling = BMP280Oversampling::X16,
				BMP280Filter filter = BMP280Filter::X16,
				BMP280StandbyDuration standbyDuration = BMP280StandbyDuration::Ms125
			) {
				// t_sb = standbyDuration
				// filter = filter
				// spi3w_en = 0
				writeRegisterValue(BMP280Register::Config, ((uint8_t) standbyDuration << 5) | ((uint8_t) filter << 2) | (uint8_t) 0);

				// osrs_t = temperatureOversampling
				// osrs_p = pressureOversampling
				// mode = mode
				writeRegisterValue(BMP280Register::Control, ((uint8_t) temperatureOversampling << 5) | ((uint8_t) pressureOversampling << 2) | (uint8_t) mode);
			}

			void readCalibrationData() {
				_calibrationDigT1 = readUint16LE(BMP280Register::DigT1);
				_calibrationDigT2 = readInt16LE(BMP280Register::DigT2);
				_calibrationDigT3 = readInt16LE(BMP280Register::DigT3);

				_calibrationDigP1 = readUint16LE(BMP280Register::DigP1);
				_calibrationDigP2 = readInt16LE(BMP280Register::DigP2);
				_calibrationDigP3 = readInt16LE(BMP280Register::DigP3);
				_calibrationDigP4 = readInt16LE(BMP280Register::DigP4);
				_calibrationDigP5 = readInt16LE(BMP280Register::DigP5);
				_calibrationDigP6 = readInt16LE(BMP280Register::DigP6);
				_calibrationDigP7 = readInt16LE(BMP280Register::DigP7);
				_calibrationDigP8 = readInt16LE(BMP280Register::DigP8);
				_calibrationDigP9 = readInt16LE(BMP280Register::DigP9);

				ESP_LOGI("BMP", "Pizda: _calibrationDigT1: %d",  _calibrationDigT1);
				ESP_LOGI("BMP", "Pizda: _calibrationDigT2: %d",  _calibrationDigT2);
				ESP_LOGI("BMP", "Pizda: _calibrationDigT3: %d",  _calibrationDigT3);
				ESP_LOGI("BMP", "Pizda: _calibrationDigP1: %d",  _calibrationDigP1);
				ESP_LOGI("BMP", "Pizda: _calibrationDigP2: %d",  _calibrationDigP2);
				ESP_LOGI("BMP", "Pizda: _calibrationDigP3: %d",  _calibrationDigP3);
				ESP_LOGI("BMP", "Pizda: _calibrationDigP4: %d",  _calibrationDigP4);
				ESP_LOGI("BMP", "Pizda: _calibrationDigP5: %d",  _calibrationDigP5);
				ESP_LOGI("BMP", "Pizda: _calibrationDigP6: %d",  _calibrationDigP6);
				ESP_LOGI("BMP", "Pizda: _calibrationDigP7: %d",  _calibrationDigP7);
				ESP_LOGI("BMP", "Pizda: _calibrationDigP8: %d",  _calibrationDigP8);
				ESP_LOGI("BMP", "Pizda: _calibrationDigP9: %d",  _calibrationDigP9);
			}

			// These bitchy compensation formulas has been taken from datasheet
			// Don't see any reason to touch them))0
			// ...
			// What are those var var lmao
			float readTemperature() {
				int32_t adc_T = readInt24BE(BMP280Register::TemperatureData);
				// Seems like this shit expects only last 20 bits from 24
				adc_T >>= 4;

				float var1, var2, T;
				var1 = (((float) adc_T) / 16384.0f - ((float) _calibrationDigT1) / 1024.0f) * ((float) _calibrationDigT2);
				var2 = ((((float) adc_T) / 131072.0f - ((float) _calibrationDigT1) / 8192.0f) *
						(((float) adc_T) / 131072.0f - ((float) _calibrationDigT1) / 8192.0f)) * ((float) _calibrationDigT3);

				_tFine = (int32_t) (var1 + var2);
				T = (var1 + var2) / 5120.0f;

				return T;
			}

			float readPressure() {
				// To process raw pressure data, we need read temperature before at least once to update "tFine"
				if (_tFine == -0xFFFF)
					readTemperature();

				int32_t adc_P = readInt24BE(BMP280Register::PressureData);
				adc_P >>= 4;

				float var1, var2, p;
				var1 = ((float)_tFine / 2.0f) - 64000.0f;
				var2 = var1 * var1 * ((float) _calibrationDigP6) / 32768.0f;
				var2 = var2 + var1 * ((float) _calibrationDigP5) * 2.0f;
				var2 = (var2/4.0f)+(((float) _calibrationDigP4) * 65536.0f);
				var1 = (((float)_calibrationDigP3) * var1 * var1 / 524288.0f + ((float) _calibrationDigP2) * var1) / 524288.0f;
				var1 = (1.0f + var1 / 32768.0f) * ((float) _calibrationDigP1);

				// avoid exception caused by division by zero
				if (var1 == 0.0f)
					return 0;

				p = 1048576.0f - (float) adc_P;
				p = (p - (var2 / 4096.0f)) * 6250.0f / var1;
				var1 = ((float)_calibrationDigP9) * p * p / 2147483648.0f;
				var2 = p * ((float)_calibrationDigP8) / 32768.0f;
				p = p + (var1 + var2 + ((float)_calibrationDigP7)) / 16.0f;

				return p;
			}

		private:
			// From datasheet:
			// The variable t_fine (signed 32 bit) carries a fine resolution temperature value over to the
			// pressure compensation formula and could be implemented as a global variable
			int32_t _tFine = -0xFFFF;

			// SPI
			gpio_num_t _misoPin;
			gpio_num_t _mosiPin;
			gpio_num_t _sckPin;
			gpio_num_t _ssPin;

			spi_device_handle_t _spiDeviceHandle {};

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

			void setupGPIO() {
				gpio_config_t config {};
				config.pin_bit_mask = 1ULL << _ssPin;
				config.mode = GPIO_MODE_OUTPUT;
				config.pull_up_en = GPIO_PULLUP_ENABLE;
				config.pull_down_en = GPIO_PULLDOWN_DISABLE;
				config.intr_type = GPIO_INTR_DISABLE;
				gpio_config(&config);
			}

			void setupSPI() {
//				// Initializing SPI bus if this hasn't been done
//				spi_bus_config_t busConfig {};
//				busConfig.mosi_io_num = _mosiPin;
//				busConfig.miso_io_num = _misoPin;
//				busConfig.sclk_io_num = _sckPin;
//				busConfig.quadwp_io_num = -1;
//				busConfig.quadhd_io_num = -1;
//				busConfig.max_transfer_sz = 320 * 240 * 2;
//
//				// May be already initialized
//				const auto result = spi_bus_initialize(SPI2_HOST, &busConfig, SPI_DMA_CH_AUTO);
//				assert(result == ESP_OK || result == ESP_ERR_INVALID_STATE);

				// Interface
				spi_device_interface_config_t interfaceConfig {};
				interfaceConfig.mode = 0;
				interfaceConfig.clock_speed_hz = 5'000;
				interfaceConfig.spics_io_num = -1;
				interfaceConfig.queue_size = 1;

				ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &interfaceConfig, &_spiDeviceHandle));
			}

			void setSlaveSelect(bool value) const {
				gpio_set_level(_ssPin, value);
			}

			void writeRegisterValue(BMP280Register reg, uint8_t value) {
				uint8_t buffer[2] {
					(uint8_t) (uint8_t(reg) & ~0x80),
					value
				};

				spi_transaction_t transaction {};
				transaction.length = 2 * 8;
				transaction.tx_buffer = buffer;

				setSlaveSelect(false);
				ESP_ERROR_CHECK(spi_device_transmit(_spiDeviceHandle, &transaction));
				setSlaveSelect(true);
			}

			void writeAndRead(BMP280Register reg, uint8_t* buffer, uint32_t readSize) {
				// Writing
				buffer[0] = (uint8_t) (uint8_t(reg) | 0x80);

				spi_transaction_t transaction {};
				transaction.length = 1 * 8;
				transaction.rx_buffer = nullptr;
				transaction.tx_buffer = buffer;
				transaction.flags = 0;

				setSlaveSelect(false);
				ESP_ERROR_CHECK(spi_device_transmit(_spiDeviceHandle, &transaction));
				setSlaveSelect(true);

				// Reading
				spi_transaction_t transaction2 = {};
				transaction2.length = readSize * 8;
				transaction2.rx_buffer = buffer;
				transaction2.tx_buffer = nullptr;
				transaction2.flags = 0;

				setSlaveSelect(false);
				ESP_ERROR_CHECK(spi_device_transmit(_spiDeviceHandle, &transaction2));
				setSlaveSelect(true);
			}

			uint16_t readUint8(BMP280Register reg) {
				uint8_t result = 0;
				writeAndRead(reg, &result, 1);

				return result;
			}

			uint16_t readUint16LE(BMP280Register reg) {
				uint8_t buffer[2];
				writeAndRead(reg, buffer, 2);

				return ((uint16_t) buffer[1] << 8) | (uint16_t) buffer[0];
			}

			int16_t readInt16LE(BMP280Register reg) {
				return (int16_t) readUint16LE(reg);
			}

			uint32_t readUint24BE(BMP280Register reg) {
				uint8_t buffer[3];
				writeAndRead(reg, buffer, 3);

				return ((uint32_t) buffer[0] << 16) | ((uint32_t) buffer[1] << 8) | (uint32_t) buffer[2];
			}

			int32_t readInt24BE(BMP280Register reg) {
				return (int32_t) readUint24BE(reg);
			}
	};
}