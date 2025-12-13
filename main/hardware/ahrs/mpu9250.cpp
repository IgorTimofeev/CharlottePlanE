#include <esp_log.h>
#include "mpu9250.h"

namespace pizda {
	bool MPU9250::setup(spi_host_device_t SPIDevice, gpio_num_t ssPin) {
		_ssPin = ssPin;

		// GPIO
		gpio_config_t GPIOConfig {};
		GPIOConfig.pin_bit_mask = 1ULL << _ssPin;
		GPIOConfig.mode = GPIO_MODE_OUTPUT;
		GPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
		GPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
		GPIOConfig.intr_type = GPIO_INTR_DISABLE;
		gpio_config(&GPIOConfig);

		/* Toggle CS pin to lock in SPI mode */
		setSlaveSelect(false);
		ets_delay_us(100'000);
		setSlaveSelect(true);
		ets_delay_us(100'000);

		// SPI interface
		spi_device_interface_config_t interfaceConfig {};
		interfaceConfig.mode = 3;
		interfaceConfig.clock_speed_hz = SPI_CFG_CLOCK_;
		interfaceConfig.spics_io_num = -1;
		interfaceConfig.queue_size = 1;
		interfaceConfig.flags = 0;

		ESP_ERROR_CHECK(spi_bus_add_device(SPIDevice, &interfaceConfig, &_SPIDeviceHandle));

		// ---------------------------------------------------------

		/* 1 MHz for config */
		spi_clock_ = SPI_CFG_CLOCK_;
		/* Select clock source to gyro */
		if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
			return false;
		}
		/* Enable I2C master mode */
		if (!WriteRegister(USER_CTRL_, I2C_MST_EN_)) {
			return false;
		}
		/* Set the I2C bus speed to 400 kHz */
		if (!WriteRegister(I2C_MST_CTRL_, I2C_MST_CLK_)) {
			return false;
		}

		ESP_LOGI("MPU", "pizda 1");

		/* Set AK8963 to power down */
		WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
		/* Reset the MPU9250 */
		WriteRegister(PWR_MGMNT_1_, H_RESET_);
		/* Wait for MPU-9250 to come back up */
		ets_delay_us(100'000);
		/* Reset the AK8963 */
		WriteAk8963Register(AK8963_CNTL2_, AK8963_RESET_);

		ESP_LOGI("MPU", "pizda 2");

		/* Select clock source to gyro */
		if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
			return false;
		}

		/* Check the WHO AM I byte */
		if (!ReadRegisters(WHOAMI_, sizeof(who_am_i_), &who_am_i_)) {
			return false;
		}

		ESP_LOGI("MPU", "who_am_i_: %d", who_am_i_);

		if ((who_am_i_ != WHOAMI_MPU9250_) && (who_am_i_ != WHOAMI_MPU9255_)) {
			return false;
		}

		/* Enable I2C master mode */
		if (!WriteRegister(USER_CTRL_, I2C_MST_EN_)) {
			return false;
		}

		/* Set the I2C bus speed to 400 kHz */
		if (!WriteRegister(I2C_MST_CTRL_, I2C_MST_CLK_)) {
			return false;
		}

		ESP_LOGI("MPU", "pizda 3");

		/* Check the AK8963 WHOAMI */
		if (!ReadAk8963Registers(AK8963_WHOAMI_, sizeof(who_am_i_), &who_am_i_)) {
			return false;
		}

		ESP_LOGI("MPU", "AK8963 WHOAMI: %d", who_am_i_);

		if (who_am_i_ != WHOAMI_AK8963_) {
			return false;
		}

		ESP_LOGI("MPU", "pizda 4");

		/* Get the magnetometer calibration */
		/* Set AK8963 to power down */
		if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_)) {
			return false;
		}

		ESP_LOGI("MPU", "pizda 4 1");

		longSleepAka();  // long wait between AK8963 mode changes
		/* Set AK8963 to FUSE ROM access */
		if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_FUSE_ROM_)) {
			return false;
		}

		longSleepAka();  // long wait between AK8963 mode changes

		ESP_LOGI("MPU", "pizda 5");

		/* Read the AK8963 ASA registers and compute magnetometer scale factors */
		if (!ReadAk8963Registers(AK8963_ASA_, sizeof(asa_buff_), asa_buff_)) {
			return false;
		}

		ESP_LOGI("MPU", "pizda 6");


		mag_scale_[0] = ((static_cast<float>(asa_buff_[0]) - 128.0f)
						 / 256.0f + 1.0f) * 4912.0f / 32760.0f;
		mag_scale_[1] = ((static_cast<float>(asa_buff_[1]) - 128.0f)
						 / 256.0f + 1.0f) * 4912.0f / 32760.0f;
		mag_scale_[2] = ((static_cast<float>(asa_buff_[2]) - 128.0f)
						 / 256.0f + 1.0f) * 4912.0f / 32760.0f;

		/* Set AK8963 to power down */
		if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_)) {
			return false;
		}
		/* Set AK8963 to 16 bit resolution, 100 Hz update rate */
		if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS2_)) {
			return false;
		}

		longSleepAka();  // long wait between AK8963 mode changes
		/* Select clock source to gyro */
		if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
			return false;
		}

		ESP_LOGI("MPU", "pizda 7");

		/* Set the accel range to 16G by default */
		if (!ConfigAccelRange(ACCEL_RANGE_16G)) {
			return false;
		}
		/* Set the gyro range to 2000DPS by default*/
		if (!ConfigGyroRange(GYRO_RANGE_2000DPS)) {
			return false;
		}
		/* Set the DLPF to 184HZ by default */
		if (!ConfigDlpfBandwidth(DLPF_BANDWIDTH_184HZ)) {
			return false;
		}

		ESP_LOGI("MPU", "pizda before ConfigSrd(0)");

		/* Set the SRD to 0 by default */
		if (!ConfigSrd(0)) {
			return false;
		}

		return true;
	}
	bool MPU9250::EnableDrdyInt() {
		spi_clock_ = SPI_CFG_CLOCK_;
		if (!WriteRegister(INT_PIN_CFG_, INT_PULSE_50US_)) {
			return false;
		}
		if (!WriteRegister(INT_ENABLE_, INT_RAW_RDY_EN_)) {
			return false;
		}
		return true;
	}
	bool MPU9250::DisableDrdyInt() {
		spi_clock_ = SPI_CFG_CLOCK_;
		if (!WriteRegister(INT_ENABLE_, INT_DISABLE_)) {
			return false;
		}
		return true;
	}
	bool MPU9250::ConfigAccelRange(const AccelRange range) {
		spi_clock_ = SPI_CFG_CLOCK_;
		/* Check input is valid and set requested range and scale */
		switch (range) {
			case ACCEL_RANGE_2G: {
				requested_accel_range_ = range;
				requested_accel_scale_ = 2.0f / 32767.5f;
				break;
			}
			case ACCEL_RANGE_4G: {
				requested_accel_range_ = range;
				requested_accel_scale_ = 4.0f / 32767.5f;
				break;
			}
			case ACCEL_RANGE_8G: {
				requested_accel_range_ = range;
				requested_accel_scale_ = 8.0f / 32767.5f;
				break;
			}
			case ACCEL_RANGE_16G: {
				requested_accel_range_ = range;
				requested_accel_scale_ = 16.0f / 32767.5f;
				break;
			}
			default: {
				return false;
			}
		}
		/* Try setting the requested range */
		if (!WriteRegister(ACCEL_CONFIG_, requested_accel_range_)) {
			return false;
		}
		/* Update stored range and scale */
		accel_range_ = requested_accel_range_;
		accel_scale_ = requested_accel_scale_;
		return true;
	}
	bool MPU9250::ConfigGyroRange(const GyroRange range) {
		spi_clock_ = SPI_CFG_CLOCK_;
		/* Check input is valid and set requested range and scale */
		switch (range) {
			case GYRO_RANGE_250DPS: {
				requested_gyro_range_ = range;
				requested_gyro_scale_ = 250.0f / 32767.5f;
				break;
			}
			case GYRO_RANGE_500DPS: {
				requested_gyro_range_ = range;
				requested_gyro_scale_ = 500.0f / 32767.5f;
				break;
			}
			case GYRO_RANGE_1000DPS: {
				requested_gyro_range_ = range;
				requested_gyro_scale_ = 1000.0f / 32767.5f;
				break;
			}
			case GYRO_RANGE_2000DPS: {
				requested_gyro_range_ = range;
				requested_gyro_scale_ = 2000.0f / 32767.5f;
				break;
			}
			default: {
				return false;
			}
		}
		/* Try setting the requested range */
		if (!WriteRegister(GYRO_CONFIG_, requested_gyro_range_)) {
			return false;
		}
		/* Update stored range and scale */
		gyro_range_ = requested_gyro_range_;
		gyro_scale_ = requested_gyro_scale_;
		return true;
	}
	bool MPU9250::ConfigSrd(const uint8_t srd) {
		ESP_LOGI("MPU", "ConfigSrd: %d", srd);

		spi_clock_ = SPI_CFG_CLOCK_;

		/* Changing the SRD to allow us to set the magnetometer successfully */
		if (!WriteRegister(SMPLRT_DIV_, 19)) {
			return false;
		}

		/* Set the magnetometer sample rate */
		if (srd > 9) {
			/* Set AK8963 to power down */
			WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);

			longSleepAka();  // long wait between AK8963 mode changes

			/* Set AK8963 to 16 bit resolution, 8 Hz update rate */
			if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS1_)) {
				return false;
			}

			longSleepAka();  // long wait between AK8963 mode changes

			if (!ReadAk8963Registers(AK8963_ST1_, sizeof(mag_data_), mag_data_)) {
				return false;
			}
		}
		else {
			/* Set AK8963 to power down */
			WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);

			longSleepAka();  // long wait between AK8963 mode changes

			/* Set AK8963 to 16 bit resolution, 100 Hz update rate */
			if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS2_)) {
				return false;
			}

			longSleepAka();  // long wait between AK8963 mode changes

			if (!ReadAk8963Registers(AK8963_ST1_, sizeof(mag_data_), mag_data_)) {
				return false;
			}
		}
		/* Set the IMU sample rate */
		if (!WriteRegister(SMPLRT_DIV_, srd)) {
			return false;
		}
		srd_ = srd;
		return true;
	}
	bool MPU9250::ConfigDlpfBandwidth(const DlpfBandwidth dlpf) {
		spi_clock_ = SPI_CFG_CLOCK_;
		/* Check input is valid and set requested dlpf */
		switch (dlpf) {
			case DLPF_BANDWIDTH_184HZ: {
				requested_dlpf_ = dlpf;
				break;
			}
			case DLPF_BANDWIDTH_92HZ: {
				requested_dlpf_ = dlpf;
				break;
			}
			case DLPF_BANDWIDTH_41HZ: {
				requested_dlpf_ = dlpf;
				break;
			}
			case DLPF_BANDWIDTH_20HZ: {
				requested_dlpf_ = dlpf;
				break;
			}
			case DLPF_BANDWIDTH_10HZ: {
				requested_dlpf_ = dlpf;
				break;
			}
			case DLPF_BANDWIDTH_5HZ: {
				requested_dlpf_ = dlpf;
				break;
			}
			default: {
				return false;
			}
		}
		/* Try setting the dlpf */
		if (!WriteRegister(ACCEL_CONFIG2_, requested_dlpf_)) {
			return false;
		}
		if (!WriteRegister(CONFIG_, requested_dlpf_)) {
			return false;
		}
		/* Update stored dlpf */
		dlpf_bandwidth_ = requested_dlpf_;
		return true;
	}
	bool MPU9250::EnableWom(int16_t threshold_mg, const WomRate wom_rate) {
		/* Check threshold in limits, 4 - 1020 mg */
		if ((threshold_mg < 4) || (threshold_mg > 1020)) {return false;}
		spi_clock_ = SPI_CFG_CLOCK_;
		/* Set AK8963 to power down */
		WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
		/* Reset the MPU9250 */
		WriteRegister(PWR_MGMNT_1_, H_RESET_);
		/* Wait for MPU-9250 to come back up */
		ets_delay_us(1'000);
		/* Cycle 0, Sleep 0, Standby 0, Internal Clock */
		if (!WriteRegister(PWR_MGMNT_1_, 0x00)) {
			return false;
		}
		/* Disable gyro measurements */
		if (!WriteRegister(PWR_MGMNT_2_, DISABLE_GYRO_)) {
			return false;
		}
		/* Set accel bandwidth to 184 Hz */
		if (!WriteRegister(ACCEL_CONFIG2_, DLPF_BANDWIDTH_184HZ)) {
			return false;
		}
		/* Set interrupt to wake on motion */
		if (!WriteRegister(INT_ENABLE_, INT_WOM_EN_)) {
			return false;
		}
		/* Enable accel hardware intelligence */
		if (!WriteRegister(MOT_DETECT_CTRL_, (ACCEL_INTEL_EN_ | ACCEL_INTEL_MODE_))) {
			return false;
		}
		/* Set the wake on motion threshold, LSB is 4 mg */
		uint8_t wom_threshold = static_cast<uint8_t>(threshold_mg /
													 static_cast<int8_t>(4));
		if (!WriteRegister(WOM_THR_, wom_threshold)) {
			return false;
		}
		/* Set the accel wakeup frequency */
		if (!WriteRegister(LP_ACCEL_ODR_, wom_rate)) {
			return false;
		}
		/* Switch to low power mode */
		if (!WriteRegister(PWR_MGMNT_1_, PWR_CYCLE_WOM_)) {
			return false;
		}
		return true;
	}
	void MPU9250::Reset() {
		spi_clock_ = SPI_CFG_CLOCK_;
		/* Set AK8963 to power down */
		WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
		/* Reset the MPU9250 */
		WriteRegister(PWR_MGMNT_1_, H_RESET_);
		/* Wait for MPU-9250 to come back up */
		ets_delay_us(1'000);
	}
	bool MPU9250::Read() {
		spi_clock_ = SPI_READ_CLOCK_;
		/* Reset the new data flags */
		new_mag_data_ = false;
		new_imu_data_ = false;

		ESP_LOGI("MPU", "eblo 1");

		/* Read the data registers */
		if (!ReadRegisters(INT_STATUS_, sizeof(data_buf_), data_buf_)) {
			return false;
		}

		/* Check if data is ready */
		new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);

		ESP_LOGI("MPU", "eblo 2");
		new_imu_data_ = true;

		if (!new_imu_data_) {
			return false;
		}

		ESP_LOGI("MPU", "eblo 3");

		/* Unpack the buffer */
		accel_cnts_[0] = static_cast<int16_t>(data_buf_[1])  << 8 | data_buf_[2];
		accel_cnts_[1] = static_cast<int16_t>(data_buf_[3])  << 8 | data_buf_[4];
		accel_cnts_[2] = static_cast<int16_t>(data_buf_[5])  << 8 | data_buf_[6];
		temp_cnts_ =     static_cast<int16_t>(data_buf_[7])  << 8 | data_buf_[8];
		gyro_cnts_[0] =  static_cast<int16_t>(data_buf_[9])  << 8 | data_buf_[10];
		gyro_cnts_[1] =  static_cast<int16_t>(data_buf_[11]) << 8 | data_buf_[12];
		gyro_cnts_[2] =  static_cast<int16_t>(data_buf_[13]) << 8 | data_buf_[14];
		new_mag_data_ = (data_buf_[15] & AK8963_DATA_RDY_INT_);
		mag_cnts_[0] =   static_cast<int16_t>(data_buf_[17]) << 8 | data_buf_[16];
		mag_cnts_[1] =   static_cast<int16_t>(data_buf_[19]) << 8 | data_buf_[18];
		mag_cnts_[2] =   static_cast<int16_t>(data_buf_[21]) << 8 | data_buf_[20];
		/* Check for mag overflow */
		mag_sensor_overflow_ = (data_buf_[22] & AK8963_HOFL_);
		if (mag_sensor_overflow_) {
			new_mag_data_ = false;
		}
		/* Convert to float values and rotate the accel / gyro axis */
		accel_[0] = static_cast<float>(accel_cnts_[1]) * accel_scale_ * G_MPS2_;
		accel_[1] = static_cast<float>(accel_cnts_[0]) * accel_scale_ * G_MPS2_;
		accel_[2] = static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f *
					G_MPS2_;
		temp_ = (static_cast<float>(temp_cnts_) - 21.0f) / TEMP_SCALE_ + 21.0f;
		gyro_[0] = static_cast<float>(gyro_cnts_[1]) * gyro_scale_ * DEG2RAD_;
		gyro_[1] = static_cast<float>(gyro_cnts_[0]) * gyro_scale_ * DEG2RAD_;
		gyro_[2] = static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f * DEG2RAD_;
		/* Only update on new data */
		if (new_mag_data_) {
			mag_[0] =   static_cast<float>(mag_cnts_[0]) * mag_scale_[0];
			mag_[1] =   static_cast<float>(mag_cnts_[1]) * mag_scale_[1];
			mag_[2] =   static_cast<float>(mag_cnts_[2]) * mag_scale_[2];
		}
		return true;
	}
	bool MPU9250::WriteRegister(const uint8_t reg, const uint8_t data) {
		spi_transaction_t transaction {};
		transaction.length = 2 * 8;
		transaction.tx_data[0] = reg;
		transaction.tx_data[1] = data;
		transaction.flags = SPI_TRANS_USE_TXDATA;

		setSlaveSelect(false);
		const auto state = spi_device_transmit(_SPIDeviceHandle, &transaction);
		setSlaveSelect(true);

		ESP_ERROR_CHECK(state);

		return state == ESP_OK;
	}
	bool MPU9250::ReadRegisters(const uint8_t reg, const uint8_t count, uint8_t * const data) {
		// 1
		spi_transaction_t transaction1 {};
		transaction1.length = 8;
		transaction1.tx_data[0] = reg | 0x80;
		transaction1.flags = SPI_TRANS_USE_TXDATA;

		setSlaveSelect(false);
		auto state = spi_device_transmit(_SPIDeviceHandle, &transaction1);

		ESP_ERROR_CHECK(state);

		if (state != ESP_OK) {
			setSlaveSelect(true);
			return false;
		}

		// 2
		spi_transaction_t transaction2 {};
		transaction2.length = count * 8;
		transaction2.rx_buffer = data;

		state = spi_device_transmit(_SPIDeviceHandle, &transaction2);
		setSlaveSelect(true);

		ESP_ERROR_CHECK(state);

		return state == ESP_OK;
	}
	bool MPU9250::WriteAk8963Register(const uint8_t reg, const uint8_t data) {
		uint8_t ret_val;
		if (!WriteRegister(I2C_SLV0_ADDR_, AK8963_I2C_ADDR_)) {
			return false;
		}
		if (!WriteRegister(I2C_SLV0_REG_, reg)) {
			return false;
		}
		if (!WriteRegister(I2C_SLV0_DO_, data)) {
			return false;
		}
		if (!WriteRegister(I2C_SLV0_CTRL_, I2C_SLV0_EN_ | sizeof(data))) {
			return false;
		}
		if (!ReadAk8963Registers(reg, sizeof(ret_val), &ret_val)) {
			return false;
		}

		ESP_LOGI("MPU", "WriteAk8963Register(), ret_val: %d, data: %d", ret_val, data);

		if (data == ret_val) {
			return true;
		} else {
			return false;
		}
	}
	bool MPU9250::ReadAk8963Registers(const uint8_t reg, const uint8_t count,
		uint8_t * const data) {
		if (!WriteRegister(I2C_SLV0_ADDR_, AK8963_I2C_ADDR_ | I2C_READ_FLAG_)) {
			return false;
		}
		if (!WriteRegister(I2C_SLV0_REG_, reg)) {
			return false;
		}
		if (!WriteRegister(I2C_SLV0_CTRL_, I2C_SLV0_EN_ | count)) {
			return false;
		}

		ets_delay_us(10'000);

		return ReadRegisters(EXT_SENS_DATA_00_, count, data);
	}
}