#pragma once

#include <driver/i2c_master.h>

namespace pizda{
	class BusHAL {
		public:
			virtual bool write(const uint8_t* buffer, size_t length) = 0;
			virtual bool read(uint8_t* buffer, size_t length) = 0;

			virtual bool read(const uint8_t reg, uint8_t* buffer, size_t length) = 0;
			virtual bool write(const uint8_t reg, uint8_t* buffer, size_t length) = 0;

			// 8
			bool writeUint8(const uint8_t reg, uint8_t value) {
				uint8_t buffer[2] {
					reg,
					value
				};

				return write(buffer, 2);
			}

			bool readUint8(const uint8_t reg, uint8_t& value) {
				return read(reg, &value, 1);
			}

			// 16 LE
			bool writeUint16LE(const uint8_t reg, uint16_t& value) {
				uint8_t buffer[3] {
					reg,
					reinterpret_cast<uint8_t*>(&value)[0],
					reinterpret_cast<uint8_t*>(&value)[1]
				};

				return write(buffer, 3);
			}

			bool readUint16LE(const uint8_t reg, uint16_t& value) {
				return read(reg, reinterpret_cast<uint8_t*>(&value), 2);
			}

			bool readInt16LE(const uint8_t reg, int16_t& value) {
				uint16_t buffer = 0;

				if (!readUint16LE(reg, buffer))
					return false;

				value = static_cast<int16_t>(buffer);

				return true;
			}

			// 16 BE
			bool writeUint16BE(const uint8_t reg, uint16_t& value) {
				uint8_t buffer[3] {
					reg,
					reinterpret_cast<uint8_t*>(&value)[1],
					reinterpret_cast<uint8_t*>(&value)[0]
				};

				return write(buffer, 3);
			}

			bool readUint16BE(const uint8_t reg, uint16_t& value) {
				if (!readUint16LE(reg, value))
					return false;

				value =
					reinterpret_cast<uint8_t*>(&value)[1]
					| reinterpret_cast<uint8_t*>(&value)[0];

				return true;
			}

			// 32 LE
			bool writeUint32LE(const uint8_t reg, uint32_t& value) {
				uint8_t buffer[5] {
					reg,
					reinterpret_cast<uint8_t*>(&value)[0],
					reinterpret_cast<uint8_t*>(&value)[1],
					reinterpret_cast<uint8_t*>(&value)[2],
					reinterpret_cast<uint8_t*>(&value)[3]
				};

				return write(buffer, 5);
			}

			bool readUint32LE(const uint8_t reg, uint32_t& value) {
				return read(reg, reinterpret_cast<uint8_t*>(&value), 4);
			}

			// 32 BE
			bool writeUint32BE(const uint8_t reg, uint32_t& value) {
				uint8_t buffer[5] {
					reg,
					reinterpret_cast<uint8_t*>(&value)[3],
					reinterpret_cast<uint8_t*>(&value)[2],
					reinterpret_cast<uint8_t*>(&value)[1],
					reinterpret_cast<uint8_t*>(&value)[0]
				};

				return write(buffer, 5);
			}

			bool readUint32BE(const uint8_t reg, uint32_t& value) {
				if (!readUint32LE(reg, value))
					return false;

				value =
					reinterpret_cast<uint8_t*>(&value)[3]
					| reinterpret_cast<uint8_t*>(&value)[2]
					| reinterpret_cast<uint8_t*>(&value)[1]
					| reinterpret_cast<uint8_t*>(&value)[0];

				return true;
			}

		protected:
			bool checkState(esp_err_t state) {
				if (state != ESP_OK) {
					ESP_ERROR_CHECK_WITHOUT_ABORT(state);
					return false;
				}

				return true;
			}
	};

	class I2CBusHAL : public BusHAL {
		public:
			bool setup(const i2c_master_bus_handle_t& bus, const uint8_t address, const uint32_t clockSpeedHz) {
				i2c_device_config_t deviceConfig {};
				deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
				deviceConfig.device_address = address;
				deviceConfig.scl_speed_hz = clockSpeedHz;

				return checkState(i2c_master_bus_add_device(bus, &deviceConfig, &_device));
			}

			bool write(const uint8_t* buffer, size_t length) override {
				return checkState(i2c_master_transmit(_device, buffer, length, -1));
			}

			bool read(uint8_t* buffer, size_t length) override {
				return checkState(i2c_master_receive(_device, buffer, length, -1));
			}

			bool write(const uint8_t reg, uint8_t* buffer, size_t length) override {
				if (!write(&reg, 1))
					return false;

				return write(buffer, length);
			}

			bool read(const uint8_t reg, uint8_t* buffer, size_t length) override {
				if (!write(&reg, 1))
					return false;

				return read(buffer, length);
			}

		private:
			i2c_master_dev_handle_t _device {};
	};
}