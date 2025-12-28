/********************************************************************
* 
* This is a library for the 6-axis gyroscope and accelerometer MPU6500.
*
* You'll find an example which should enable you to use the library.
*
* You are free to use it, change it or build on it. In case you like
* it, it would be cool if you give it a star.
*
* If you find bugs, please inform me!
*
* Written by Wolfgang (Wolle) Ewald
*
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
*
*********************************************************************/

#include "MPU9250.h"

#include <cstdint>
#include <cmath>
#include <esp_log.h>

namespace pizda {
	bool MPU9250::setup(BusStream* bus) {
		_bus = bus;

		resetMPU9250();
		delayMs(10);
		writeMPU9250Register(REGISTER_INT_PIN_CFG, REGISTER_VALUE_BYPASS_EN);  // Bypass Enable
		delayMs(10);

		// Who am I check
		const auto whoAmI = getWhoAmI();

		if (whoAmI != WHO_AM_I_CODE) {
			ESP_LOGE(_logTag, "Unknown whoAmI value: %d", whoAmI);

			return false;
		}

		setSleepPowerMode(false);

		if (!setupMagnetometer()) {
			ESP_LOGE(_logTag, "Mag initialize failed");
			return false;
		}

		return true;
	}

	uint8_t MPU9250::getWhoAmI() {
		return readMPU9250Register8(REGISTER_WHO_AM_I);
	}

//	void MPU9250::calibrateAccAndGyr() {
//		// Highest resolution
//		setGyrRange(MPU9250_GYRO_RANGE_250);
//		setAccRange(MPU9250_ACC_RANGE_2G);
//
//		// Lowest noise
//		enableAccDLPF();
//		setAccDLPF(MPU9250_DLPF_6);
//
//		enableGyrDLPF();
//		setGyrDLPF(MPU9250_DLPF_6);
//
//		delayMs(100);
//
//		// Starting
//		Vector3F accAcc {};
//		Vector3F gyroAcc {};
//
//		constexpr static uint16_t iterations = 100;
//
//		for (uint16_t i = 0; i < iterations; i++) {
//			accAcc += readRawAccValues();
//			gyroAcc += readRawGyroValues();
//
//			delayMs(1);
//		}
//
//		accAcc /= iterations;
//		gyroAcc /= iterations;
//
//
//		accOffsetVal = accAcc;
//		gyrOffsetVal = gyroAcc;
//	}

	void MPU9250::setGyroDLPF(MPU9250_dlpf dlpf) {
		uint8_t regVal = readMPU9250Register8(REGISTER_CONFIG);
		regVal &= 0xF8;
		regVal |= dlpf;
		writeMPU9250Register(REGISTER_CONFIG, regVal);
	}

	void MPU9250::setSRD(uint8_t splRateDiv) {
		writeMPU9250Register(REGISTER_SMPLRT_DIV, splRateDiv);
	}

	void MPU9250::setGyroRange(MPU9250_gyroRange gyroRange) {
		uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
		regVal &= 0xE7;
		regVal |= (gyroRange << 3);
		writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);

		gyroScaleFactor = static_cast<float>(1 << gyroRange) * 250.f / 32768.0f;
	}

	void MPU9250::enableGyroDLPF() {
		uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
		regVal &= 0xFC;
		writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
	}

	void MPU9250::disableGyroDLPF(MPU9250_bw_wo_dlpf bw) {
		uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
		regVal &= 0xFC;
		regVal |= bw;
		writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
	}

	void MPU9250::setAccelRange(MPU9250_accRange accRange) {
		uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG);
		regVal &= 0xE7;
		regVal |= (accRange << 3);
		writeMPU9250Register(REGISTER_ACCEL_CONFIG, regVal);

		accelScaleFactor = (static_cast<float>(1 << accRange) * 2.f / 32768.0f);
	}

	void MPU9250::enableAccelDLPF() {
		uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
		regVal &= ~8;
		writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
	}

	void MPU9250::disableAccelDLPF() {
		uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
		regVal |= 8;
		writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
	}

	void MPU9250::setAccelDLPF(MPU9250_dlpf dlpf) {
		uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
		regVal &= 0xF8;
		regVal |= dlpf;
		writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
	}

	void MPU9250::setLowPowerAccelDataRate(MPU9250_lpAccODR lpaodr) {
		writeMPU9250Register(REGISTER_LP_ACCEL_ODR, lpaodr);
	}

	void MPU9250::enableAccelAxes(MPU9250_xyzEn enable) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_2);
		regVal &= ~(0x38);
		regVal |= (enable << 3);
		writeMPU9250Register(REGISTER_PWR_MGMT_2, regVal);
	}

	void MPU9250::enableGyroAxes(MPU9250_xyzEn enable) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_2);
		regVal &= ~(0x07);
		regVal |= enable;
		writeMPU9250Register(REGISTER_PWR_MGMT_2, regVal);
	}

	Vector3F MPU9250::getAccelData(uint8_t* buffer) {
		const auto x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
		const auto y = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
		const auto z = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);

		return Vector3F {
			static_cast<float>(x) * accelScaleFactor,
			static_cast<float>(y) * accelScaleFactor,
			static_cast<float>(z) * accelScaleFactor
		};
	}

	Vector3F MPU9250::getAccelData() {
		uint8_t buffer[6];
		_bus->read(REGISTER_ACCEL_OUT | 0x80, buffer, 6);

		return getAccelData(buffer);
	}

	Vector3F MPU9250::getGyroData(uint8_t* buffer) {
		const auto x = static_cast<int16_t>((buffer[0] << 8) | buffer[1]);
		const auto y = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
		const auto z = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);

		return {
			static_cast<float>(x) * gyroScaleFactor,
			static_cast<float>(y) * gyroScaleFactor,
			static_cast<float>(z) * gyroScaleFactor
		};
	}

	Vector3F MPU9250::getGyroData() {
		uint8_t buffer[6];
		_bus->read(REGISTER_GYRO_OUT | 0x80, buffer, 6);

		return getGyroData(buffer);
	}

	float MPU9250::getTemperature() {
		int16_t value = 0;
		_bus->readInt16BE(REGISTER_TEMP_OUT | 0x80, value);

		return (value * 1.0 - ROOM_TEMPERATURE_OFFSET) / TEMPERATURE_SENSITIVITY + 21.0;
	}

	/********* Power, Sleep, Standby *********/

	void MPU9250::setSleepPowerMode(bool sleep) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
		if (sleep) {
			regVal |= 0x40;
		} else {
			regVal &= ~(0x40);
		}
		writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
	}

	void MPU9250::setCyclePowerMode(bool cycle) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
		if (cycle) {
			regVal |= 0x20;
		} else {
			regVal &= ~(0x20);
		}
		writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
	}

	void MPU9250::setStandbyPowerMode(bool gyroStandby) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
		if (gyroStandby) {
			regVal |= 0x10;
		} else {
			regVal &= ~(0x10);
		}
		writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
	}


/************** Interrupts ***************/

	void MPU9250::setIntPinPolarity(MPU9250_intPinPol pol) {
		uint8_t regVal = readMPU9250Register8(REGISTER_INT_PIN_CFG);
		if (pol) {
			regVal |= 0x80;
		} else {
			regVal &= ~(0x80);
		}
		writeMPU9250Register(REGISTER_INT_PIN_CFG, regVal);
	}

	void MPU9250::enableIntLatch(bool latch) {
		uint8_t regVal = readMPU9250Register8(REGISTER_INT_PIN_CFG);
		if (latch) {
			regVal |= 0x20;
		} else {
			regVal &= ~(0x20);
		}
		writeMPU9250Register(REGISTER_INT_PIN_CFG, regVal);
	}

	void MPU9250::enableClearIntByAnyRead(bool clearByAnyRead) {
		uint8_t regVal = readMPU9250Register8(REGISTER_INT_PIN_CFG);
		if (clearByAnyRead) {
			regVal |= 0x10;
		} else {
			regVal &= ~(0x10);
		}
		writeMPU9250Register(REGISTER_INT_PIN_CFG, regVal);
	}

	void MPU9250::enableInterrupt(MPU9250_intType intType) {
		uint8_t regVal = readMPU9250Register8(REGISTER_INT_ENABLE);
		regVal |= intType;
		writeMPU9250Register(REGISTER_INT_ENABLE, regVal);
	}

	void MPU9250::disableInterrupt(MPU9250_intType intType) {
		uint8_t regVal = readMPU9250Register8(REGISTER_INT_ENABLE);
		regVal &= ~intType;
		writeMPU9250Register(REGISTER_INT_ENABLE, regVal);
	}

	bool MPU9250::checkInterrupt(uint8_t source, MPU9250_intType type) {
		source &= type;
		return source;
	}

	uint8_t MPU9250::readAndClearInterruptStatus() {
		uint8_t regVal = readMPU9250Register8(REGISTER_INT_STATUS);
		return regVal;
	}

	void MPU9250::setWakeOnMotionThreshold(uint8_t womthresh) {
		writeMPU9250Register(REGISTER_WOM_THR, womthresh);
	}

	void MPU9250::enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn) {
		uint8_t regVal = 0;
		if (womEn) {
			regVal |= 0x80;
		}
		if (womCompEn) {
			regVal |= 0x40;
		}
		writeMPU9250Register(REGISTER_MOT_DET_CTRL, regVal);
	}

/***************** FIFO ******************/

	void MPU9250::setFIFODataSource(MPU9250_fifo_data_source dataSourceBitMask) {
		writeMPU9250Register(REGISTER_FIFO_EN, dataSourceBitMask);
	}

	void MPU9250::enableFIFO() {
		uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
		regVal |= 0x40;
		writeMPU9250Register(REGISTER_USER_CTRL, regVal);
	}

	void MPU9250::disableFIFO() {
		uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
		regVal &= ~(0x40);
		writeMPU9250Register(REGISTER_USER_CTRL, regVal);
	}

	void MPU9250::resetFIFO() {
		uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
		regVal |= 0x04;
		writeMPU9250Register(REGISTER_USER_CTRL, regVal);
	}

	uint16_t MPU9250::getFIFOCount() {
		uint16_t value = 0;
		_bus->readUint16BE(REGISTER_FIFO_COUNT | 0x80, value);

		return value;
	}

	void MPU9250::setFIFOMode(MPU9250_fifoMode mode) {
		uint8_t regVal = readMPU9250Register8(REGISTER_CONFIG);

		if (mode) {
			regVal |= 0x40;
		}
		else {
			regVal &= ~(0x40);
		}

		writeMPU9250Register(REGISTER_CONFIG, regVal);
	}

	void MPU9250::resetMPU9250() {
		writeMPU9250Register(REGISTER_PWR_MGMT_1, REGISTER_VALUE_RESET);
		delayMs(10);  // wait for registers to reset
	}

	void MPU9250::enableI2CMaster() {
		uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
		regVal |= REGISTER_VALUE_I2C_MST_EN;

		// Enable I2C master
		writeMPU9250Register(REGISTER_USER_CTRL, regVal);

		// Set I2C slave clock
		// Value | Clock speed | Clock divider
		// 0     | 348 kHz     | 23
		// 1     | 333 kHz     | 24
		// 2     | 320 kHz     | 25
		// 3     | 308 kHz     | 26
		// 4     | 296 kHz     | 27
		// 5     | 286 kHz     | 28
		// 6     | 276 kHz     | 29
		// 7     | 267 kHz     | 30
		// 8     | 258 kHz     | 31
		// 9     | 500 kHz     | 16
		// 10    | 471 kHz     | 17
		// 11    | 444 kHz     | 18
		// 12    | 421 kHz     | 19
		// 13    | 400 kHz     | 20
		// 14    | 381 kHz     | 21
		// 15    | 364 kHz     | 22
		writeMPU9250Register(REGISTER_I2C_MST_CTRL, 0);

//		// Enable I2C slave 0 read delay
//		// Bit 0 = I2C_SLV0_DLY_EN
//		// Bit 1 = I2C_SLV1_DLY_EN
//		writeMPU9250Register(REGISTER_I2C_MST_DELAY_CTRL, 0b00000001);
//
//		// Set all slaves delay for 4 periods (20 ms with 200 Hz master sample rate)
//		// SLV4 is common controls channel for slaves 0-3
//		// Enabled | delay in periods
//		uint8_t delayValue = 2;
//		writeMPU9250Register(REGISTER_I2C_SLV4_CTRL, 0b1000'0000 | delayValue);

		delayMs(10);
	}

	void MPU9250::writeMPU9250Register(uint8_t reg, uint8_t val) {
		_bus->writeUint8(reg, val);
	}

	uint8_t MPU9250::readMPU9250Register8(uint8_t reg) {
		uint8_t result = 0;

		_bus->readUint8(reg | 0x80, result);

		return result;
	}

/************** Magnetometer **************/

	bool MPU9250::setupMagnetometer() {
		enableI2CMaster();
		resetAK8963();

		const auto whoAmI = readWhoAmIMag();

		if (whoAmI != MAGNETOMETER_WHO_AM_I_CODE) {
			ESP_LOGE(_logTag, "Unknown mag WhoAmI value: %d", whoAmI);

			return false;
		}

		setMagOpMode(AK8963_FUSE_ROM_ACC_MODE);
		delayMs(10);
		raedAK8963ASAVals();
		delayMs(10);
		setAK896316Bit();
		delayMs(10);
		setMagOpMode(AK8963_CONT_MODE_100HZ);
		delayMs(10);

		return true;
	}

	void pizda() {

	}

	uint8_t MPU9250::readWhoAmIMag() {
		return readAK8963Register8(REGISTER_AK8963_WIA);
	}

	void MPU9250::setMagOpMode(AK8963_opMode opMode) {
		uint8_t regVal = readAK8963Register8(REGISTER_AK8963_CNTL_1);
		regVal &= 0xF0;
		regVal |= opMode;
		writeAK8963Register(REGISTER_AK8963_CNTL_1, regVal);
		delayMs(10);
		if (opMode != AK8963_PWR_DOWN) {
			enableAK8963DataRead(REGISTER_AK8963_HXL, 0x08);
		}
	}

/************************************************1
     Private Functions
*************************************************/

	void MPU9250::enableAK8963DataRead(uint8_t reg, uint8_t bytes) {
		writeMPU9250Register(REGISTER_I2C_SLV0_ADDR, MAGNETOMETER_I2C_ADDRESS | REGISTER_VALUE_AK8963_READ); // read AK8963
		writeMPU9250Register(REGISTER_I2C_SLV0_REG, reg); // define AK8963 register to be read
		writeMPU9250Register(REGISTER_I2C_SLV0_CTRL, 0b1000'0000 | bytes); // Enable read | number of byte
		delayMs(10);
	}

	void MPU9250::resetAK8963() {
		writeAK8963Register(REGISTER_AK8963_CNTL_2, 0x01);
		delayMs(100);
	}

	Vector3F MPU9250::getMagData(uint8_t* buffer) {
		const auto x = static_cast<int16_t>((buffer[1] << 8) | buffer[0]);
		const auto y = static_cast<int16_t>((buffer[3] << 8) | buffer[2]);
		const auto z = static_cast<int16_t>((buffer[5] << 8) | buffer[4]);

		return {
			x * magScaleFactor * magASAFactor.getX(),
			y * magScaleFactor * magASAFactor.getY(),
			z * magScaleFactor * magASAFactor.getZ()
		};
	}

	Vector3F MPU9250::getMagData() {
		uint8_t rawData[6];
		readAK8963Data(rawData);

		return getMagData(rawData);
	}

	void MPU9250::getFIFOData(uint8_t* buffer, uint16_t count) {
		_bus->read(REGISTER_FIFO_R_W | 0x80, buffer, count);
	}

	void MPU9250::raedAK8963ASAVals() {
		uint8_t rawCorr = 0;
		rawCorr = readAK8963Register8(REGISTER_AK8963_ASAX);
		magASAFactor.setX((0.5 * (rawCorr - 128) / 128.0) + 1.0);
		rawCorr = readAK8963Register8(REGISTER_AK8963_ASAY);
		magASAFactor.setY((0.5 * (rawCorr - 128) / 128.0) + 1.0);
		rawCorr = readAK8963Register8(REGISTER_AK8963_ASAZ);
		magASAFactor.setZ((0.5 * (rawCorr - 128) / 128.0) + 1.0);

//		ESP_LOGI(_logTag, "ASA vals: %f, %f, %f", magASAFactor.getX(), magASAFactor.getY(), magASAFactor.getZ());
	}

	void MPU9250::writeAK8963Register(uint8_t reg, uint8_t val) {
		writeMPU9250Register(REGISTER_I2C_SLV0_ADDR, MAGNETOMETER_I2C_ADDRESS); // write AK8963
		writeMPU9250Register(REGISTER_I2C_SLV0_REG, reg); // define AK8963 register to be written to
		writeMPU9250Register(REGISTER_I2C_SLV0_DO, val);
	}

	uint8_t MPU9250::readAK8963Register8(uint8_t reg) {
		enableAK8963DataRead(reg, 0x01);
		uint8_t const regVal = readMPU9250Register8(REGISTER_EXT_SLV_SENS_DATA_00);
		enableAK8963DataRead(REGISTER_AK8963_HXL, 0x08);

		return regVal;
	}

	void MPU9250::readAK8963Data(uint8_t* buf) {
		_bus->read(REGISTER_EXT_SLV_SENS_DATA_00 | 0x80, buf, 6);

//	if(!useSPI){
//		_wire->beginTransmission(i2cAddress);
//		_wire->write(MPU9250::REGISTER_EXT_SLV_SENS_DATA_00);
//		_wire->endTransmission(false);
//		_wire->requestFrom(i2cAddress,(uint8_t)6);
//		if(_wire->available()){
//			for(int i=0; i<6; i++){
//				buf[i] = _wire->read();
//			}
//		}
//	}
//	else{
//		uint8_t reg = MPU9250::REGISTER_EXT_SLV_SENS_DATA_00 | 0x80;
//		_spi->beginTransaction(mySPISettings);
//		digitalWrite(csPin, LOW);
//		_spi->transfer(reg);
//		for(int i=0; i<6; i++){
//			buf[i] = _spi->transfer(0x00);
//		}
//		digitalWrite(csPin, HIGH);
//		_spi->endTransaction();
//	}
	}

	void MPU9250::setAK896316Bit() {
		uint8_t regVal = readAK8963Register8(REGISTER_AK8963_CNTL_1);
		regVal |= REGISTER_VALUE_AK8963_16_BIT;
		writeAK8963Register(REGISTER_AK8963_CNTL_1, regVal);
	}

	uint8_t MPU9250::readAK8963Status2Register() {
		return readAK8963Register8(REGISTER_AK8963_STATUS_2);
	}

	void MPU9250::delayMs(uint32_t ms) {
		vTaskDelay(ms <= portTICK_PERIOD_MS ? portTICK_PERIOD_MS : pdMS_TO_TICKS(ms));
	}
}
