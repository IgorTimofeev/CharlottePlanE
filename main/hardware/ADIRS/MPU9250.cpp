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
	bool MPU9250::setup(i2c_master_bus_handle_t I2CBusHandle, uint8_t I2CAddress) {
		// I2C
		i2c_device_config_t I2CDeviceConfig {};
		I2CDeviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		I2CDeviceConfig.device_address = I2CAddress;
		I2CDeviceConfig.scl_speed_hz = 400'000;

		const auto state = i2c_master_bus_add_device(I2CBusHandle, &I2CDeviceConfig, &_I2CDeviceHandle);

		if (state != ESP_OK) {
			ESP_ERROR_CHECK_WITHOUT_ABORT(state);
			return false;
		}

		reset_MPU9250();
		delayMs(10);
		writeMPU9250Register(REGISTER_INT_PIN_CFG, REGISTER_VALUE_BYPASS_EN);  // Bypass Enable
		delayMs(10);

		// Who am I check
		const auto whoAmI = readWhoAmI();

		if (whoAmI != WHO_AM_I_CODE) {
			ESP_LOGE(_logTag, "Unknown whoAmI value: %d", whoAmI);

			return false;
		}

		sleep(false);

		if (!setupMagnetometer()) {
			ESP_LOGE(_logTag, "Mag setup failed");
			return false;
		}

		return true;
	}

	uint8_t MPU9250::readWhoAmI() {
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

	void MPU9250::setGyrDLPF(MPU9250_dlpf dlpf) {
		uint8_t regVal = readMPU9250Register8(REGISTER_CONFIG);
		regVal &= 0xF8;
		regVal |= dlpf;
		writeMPU9250Register(REGISTER_CONFIG, regVal);
	}

	void MPU9250::setSRD(uint8_t splRateDiv) {
		writeMPU9250Register(REGISTER_SMPLRT_DIV, splRateDiv);
	}

	void MPU9250::setGyrRange(MPU9250_gyroRange gyroRange) {
		uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
		regVal &= 0xE7;
		regVal |= (gyroRange << 3);
		writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);

		gyrRangeFactor = static_cast<float>(1 << gyroRange) * 250.f / 32768.0f;
	}

	void MPU9250::enableGyrDLPF() {
		uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
		regVal &= 0xFC;
		writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
	}

	void MPU9250::disableGyrDLPF(MPU9250_bw_wo_dlpf bw) {
		uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
		regVal &= 0xFC;
		regVal |= bw;
		writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
	}

	void MPU9250::setAccRange(MPU9250_accRange accRange) {
		uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG);
		regVal &= 0xE7;
		regVal |= (accRange << 3);
		writeMPU9250Register(REGISTER_ACCEL_CONFIG, regVal);

		accRangeFactor = (static_cast<float>(1 << accRange) * 2.f / 32768.0f);
	}

	void MPU9250::enableAccDLPF() {
		uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
		regVal &= ~8;
		writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
	}

	void MPU9250::disableAccDLPF() {
		uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
		regVal |= 8;
		writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
	}

	void MPU9250::setAccDLPF(MPU9250_dlpf dlpf) {
		uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
		regVal &= 0xF8;
		regVal |= dlpf;
		writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
	}

	void MPU9250::setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr) {
		writeMPU9250Register(REGISTER_LP_ACCEL_ODR, lpaodr);
	}

	void MPU9250::enableAccAxes(MPU9250_xyzEn enable) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_2);
		regVal &= ~(0x38);
		regVal |= (enable << 3);
		writeMPU9250Register(REGISTER_PWR_MGMT_2, regVal);
	}

	void MPU9250::enableGyrAxes(MPU9250_xyzEn enable) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_2);
		regVal &= ~(0x07);
		regVal |= enable;
		writeMPU9250Register(REGISTER_PWR_MGMT_2, regVal);
	}

/************* x,y,z results *************/

	Vector3F MPU9250::readAccValues() {
		uint8_t values[6];
		readMPU9250Register3x16(REGISTER_ACCEL_OUT, values);

		const auto x = static_cast<int16_t>((values[0] << 8) | values[1]);
		const auto y = static_cast<int16_t>((values[2] << 8) | values[3]);
		const auto z = static_cast<int16_t>((values[4] << 8) | values[5]);

		return Vector3F {
			static_cast<float>(x) * accRangeFactor,
			static_cast<float>(y) * accRangeFactor,
			static_cast<float>(z) * accRangeFactor
		};
	}

	Vector3F MPU9250::readAccValuesFromFIFO() {
		return readVector3ValueFromFIFO() * accRangeFactor;
	}

	Vector3F MPU9250::readGyroValues() {
		uint8_t values[6];
		readMPU9250Register3x16(REGISTER_GYRO_OUT, values);

		const auto x = static_cast<int16_t>((values[0] << 8) | values[1]);
		const auto y = static_cast<int16_t>((values[2] << 8) | values[3]);
		const auto z = static_cast<int16_t>((values[4] << 8) | values[5]);

		return {
			static_cast<float>(x) * gyrRangeFactor,
			static_cast<float>(y) * gyrRangeFactor,
			static_cast<float>(z) * gyrRangeFactor
		};
	}

	Vector3F MPU9250::readGyroValuesFromFIFO() {
		return readVector3ValueFromFIFO() * gyrRangeFactor;
	}

	float MPU9250::readTemperature() {
		int16_t regVal16 = readMPU9250Register16(REGISTER_TEMP_OUT);
		float tmp = (regVal16 * 1.0 - ROOM_TEMPERATURE_OFFSET) / TEMPERATURE_SENSITIVITY + 21.0;
		return tmp;
	}

	/********* Power, Sleep, Standby *********/

	void MPU9250::sleep(bool sleep) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
		if (sleep) {
			regVal |= 0x40;
		} else {
			regVal &= ~(0x40);
		}
		writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
	}

	void MPU9250::enableCycle(bool cycle) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
		if (cycle) {
			regVal |= 0x20;
		} else {
			regVal &= ~(0x20);
		}
		writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
	}

	void MPU9250::enableGyrStandby(bool gyroStandby) {
		uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
		if (gyroStandby) {
			regVal |= 0x10;
		} else {
			regVal &= ~(0x10);
		}
		writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
	}


/******** Angles and Orientation *********/

	Vector3F MPU9250::getAngles() {
		Vector3F angleVal {};
		auto gVal = readAccValues();

		if (gVal.getX() > 1.0) {
			gVal.setX(1.0);
		} else if (gVal.getX() -1.0) {
			gVal.setX(-1.0);
		}
		angleVal.setX((asin(gVal.getX())) * 57.296);

		if (gVal.getY() > 1.0) {
			gVal.setY(1.0);
		} else if (gVal.getY() < -1.0) {
			gVal.setY(-1.0);
		}
		angleVal.setY((asin(gVal.getY())) * 57.296);

		if (gVal.getZ() > 1.0) {
			gVal.setZ(1.0);
		} else if (gVal.getZ() < -1.0) {
			gVal.setZ(-1.0);
		}
		angleVal.setZ((asin(gVal.getZ())) * 57.296);

		return angleVal;
	}

	MPU9250_orientation MPU9250::getOrientation() {
		const auto angleVal = getAngles();
		auto orientation = MPU9250_FLAT;

		if (abs(angleVal.getX()) < 45) {      // |x| < 45
			if (abs(angleVal.getY()) < 45) {      // |y| < 45
				if (angleVal.getZ() > 0) {          //  z  > 0
					orientation = MPU9250_FLAT;
				} else {                        //  z  < 0
					orientation = MPU9250_FLAT_1;
				}
			} else {                         // |y| > 45
				if (angleVal.getY() > 0) {         //  y  > 0
					orientation = MPU9250_XY;
				} else {                       //  y  < 0
					orientation = MPU9250_XY_1;
				}
			}
		} else {                           // |x| >= 45
			if (angleVal.getX() > 0) {           //  x  >  0
				orientation = MPU9250_YX;
			} else {                       //  x  <  0
				orientation = MPU9250_YX_1;
			}
		}
		return orientation;
	}

//String MPU9250::getOrientationAsString(){
//    MPU9250_orientation orientation = getOrientation();
//    String orientationAsString = "";
//    switch(orientation){
//        case MPU9250_FLAT:      orientationAsString = "z up";   break;
//        case MPU9250_FLAT_1:    orientationAsString = "z down"; break;
//        case MPU9250_XY:        orientationAsString = "y up";   break;
//        case MPU9250_XY_1:      orientationAsString = "y down"; break;
//        case MPU9250_YX:        orientationAsString = "x up";   break;
//        case MPU9250_YX_1:      orientationAsString = "x down"; break;
//    }
//    return orientationAsString;
//}

	float MPU9250::getPitch() {
		Vector3F angleVal = getAngles();
		float pitch = (atan2(-angleVal.getX(), sqrt(abs((angleVal.getY() * angleVal.getY() + angleVal.getZ() * angleVal.getZ())))) * 180.0) / M_PI;
		return pitch;
	}


	float MPU9250::getRoll() {
		Vector3F angleVal = getAngles();
		float roll = (atan2(angleVal.getY(), angleVal.getY()) * 180.0) / M_PI;
		return roll;
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

/* fifo is a byte which defines the data stored in the FIFO
 * It is structured as:
 * Bit 7 = TEMP,              Bit 6 = GYRO_X,  Bit 5 = GYRO_Y   Bit 4 = GYRO_Z,
 * Bit 3 = ACCEL (all axes), Bit 2 = SLAVE_2, Bit 1 = SLAVE_1, Bit 0 = SLAVE_0;
 * e.g. 0b11001001 => TEMP, GYRO_X, ACCEL, SLAVE0 are enabled
 */
	void MPU9250::startFIFO(MPU9250_fifo_type fifo) {
		fifoType = fifo;
		writeMPU9250Register(REGISTER_FIFO_EN, fifoType);
	}

	void MPU9250::stopFIFO() {
		writeMPU9250Register(REGISTER_FIFO_EN, 0);
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

	int16_t MPU9250::readFIFOCount() {
		uint16_t regVal16 = (uint16_t) readMPU9250Register16(REGISTER_FIFO_COUNT);
		return regVal16;
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

	int16_t MPU9250::readFIFODataSetsCount() {
		auto dataSetsCount = readFIFOCount();

		if ((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)) {
			dataSetsCount /= 6;
		}
		else if (fifoType == MPU9250_FIFO_ACC_GYR) {
			dataSetsCount /= 12;
		}

		return dataSetsCount;
	}

	void MPU9250::findFIFOBegin() {
		int16_t count = readFIFOCount();

		if ((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)) {
			if (count > 510) {
				for (int i = 0; i < 2; i++) {
					readMPU9250Register8(REGISTER_FIFO_R_W);
				}
			}
		}
		else if (fifoType == MPU9250_FIFO_ACC_GYR) {
			if (count > 504) {
				for (int i = 0; i < 8; i++) {
					readMPU9250Register8(REGISTER_FIFO_R_W);
				}
			}
		}
	}

/************************************************
     Private Functions
*************************************************/

	void MPU9250::reset_MPU9250() {
		writeMPU9250Register(REGISTER_PWR_MGMT_1, REGISTER_VALUE_RESET);
		delayMs(10);  // wait for registers to reset
	}

	void MPU9250::enableI2CMaster() {
		uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
		regVal |= REGISTER_VALUE_I2C_MST_EN;
		writeMPU9250Register(REGISTER_USER_CTRL, regVal); //enable I2C master
		writeMPU9250Register(REGISTER_I2C_MST_CTRL, 0x00); // set I2C clock to 400 kHz
		delayMs(10);
	}

	void MPU9250::writeMPU9250Register(uint8_t reg, uint8_t val) {
		uint8_t buffer[2]{
			reg,
			val
		};

		ESP_ERROR_CHECK(i2c_master_transmit(_I2CDeviceHandle, buffer, 2, -1));
	}

	uint8_t MPU9250::readMPU9250Register8(uint8_t reg) {
		const uint8_t cmd = reg | 0x80;
		ESP_ERROR_CHECK(i2c_master_transmit(_I2CDeviceHandle, &cmd, 1, -1));

		uint8_t result = 0;
		ESP_ERROR_CHECK(i2c_master_receive(_I2CDeviceHandle, &result, 1, -1));

		return result;
	}

	int16_t MPU9250::readMPU9250Register16(uint8_t reg) {
		const uint8_t cmd = reg | 0x80;
		ESP_ERROR_CHECK(i2c_master_transmit(_I2CDeviceHandle, &cmd, 1, -1));

		uint8_t result[2];
		ESP_ERROR_CHECK(i2c_master_receive(_I2CDeviceHandle, result, 2, -1));

		return (result[0] << 8) | result[0];

//	reg |= 0x80;
//	_spi->beginTransaction(mySPISettings);
//	digitalWrite(csPin, LOW);
//	_spi->transfer(reg);
//	MSByte = _spi->transfer(0x00);
//	LSByte = _spi->transfer(0x00);
//	digitalWrite(csPin, HIGH);
//	_spi->endTransaction();
//    regValue = (MSByte<<8) + LSByte;
//    return regValue;
	}

	void MPU9250::readMPU9250Register3x16(uint8_t reg, uint8_t* buffer) {
		const uint8_t cmd = reg | 0x80;
		ESP_ERROR_CHECK(i2c_master_transmit(_I2CDeviceHandle, &cmd, 1, -1));

		ESP_ERROR_CHECK(i2c_master_receive(_I2CDeviceHandle, buffer, 6, -1));

//    if(!useSPI){
//        _wire->beginTransmission(i2cAddress);
//        _wire->write(reg);
//        _wire->endTransmission(false);
//        _wire->requestFrom(i2cAddress,(uint8_t)6);
//        if(_wire->available()){
//            for(int i=0; i<6; i++){
//                buf[i] = _wire->read();
//            }
//        }
//    }
//    else{
//        reg |= 0x80;
//        _spi->beginTransaction(mySPISettings);
//        digitalWrite(csPin, LOW);
//        _spi->transfer(reg);
//        for(int i=0; i<6; i++){
//                buf[i] = _spi->transfer(0x00);
//        }
//        digitalWrite(csPin, HIGH);
//        _spi->endTransaction();
//    }
	}

	Vector3F MPU9250::readVector3ValueFromFIFO() {
		uint8_t values[6];

		const uint8_t cmd = REGISTER_FIFO_R_W | 0x80;
		ESP_ERROR_CHECK(i2c_master_transmit(_I2CDeviceHandle, &cmd, 1, -1));

		ESP_ERROR_CHECK(i2c_master_receive(_I2CDeviceHandle, values, 6, -1));

//    if(!useSPI){
//        _wire->beginTransmission(i2cAddress);
//        _wire->write(REGISTER_FIFO_R_W);
//        _wire->endTransmission(false);
//        _wire->requestFrom(i2cAddress,(uint8_t)6);
//        if(_wire->available()){
//            for(int i=0; i<6; i++){
//                fifoTriple[i] = _wire->read();
//            }
//        }
//    }
//    else{
//        uint8_t reg = REGISTER_FIFO_R_W | 0x80;
//        _spi->beginTransaction(mySPISettings);
//        digitalWrite(csPin, LOW);
//        _spi->transfer(reg);
//        for(int i=0; i<6; i++){
//                fifoTriple[i] = _spi->transfer(0x00);
//        }
//        digitalWrite(csPin, HIGH);
//        _spi->endTransaction();
//    }

		return {
			static_cast<float>((int16_t) ((values[0] << 8) + values[1])),
			static_cast<float>((int16_t) ((values[2] << 8) + values[3])),
			static_cast<float>((int16_t) ((values[4] << 8) + values[5]))
		};
	}

/************** Magnetometer **************/

	bool MPU9250::setupMagnetometer() {
		enableI2CMaster();
		resetMagnetometer();

		const auto whoAmI = readWhoAmIMag();

		if (whoAmI != MAGNETOMETER_WHO_AM_I_CODE) {
			ESP_LOGE(_logTag, "Unknown mag WhoAmI value: %d", whoAmI);

			return false;
		}

		setMagOpMode(AK8963_FUSE_ROM_ACC_MODE);
		delayMs(10);
		getAsaVals();
		delayMs(10);
		setMagnetometer16Bit();
		delayMs(10);
		setMagOpMode(AK8963_CONT_MODE_8HZ);
		delayMs(10);

		return true;
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
			enableMagDataRead(REGISTER_AK8963_HXL, 0x08);
		}
	}

	void MPU9250::startMagMeasurement() {
		setMagOpMode(AK8963_TRIGGER_MODE);
		delayMs(200);
	}

/************************************************
     Private Functions
*************************************************/

	void MPU9250::enableMagDataRead(uint8_t reg, uint8_t bytes) {
		writeMPU9250Register(REGISTER_I2C_SLV0_ADDR,
			MAGNETOMETER_I2C_ADDRESS | REGISTER_VALUE_AK8963_READ); // read AK8963
		writeMPU9250Register(REGISTER_I2C_SLV0_REG, reg); // define AK8963 register to be read
		writeMPU9250Register(REGISTER_I2C_SLV0_CTRL, 0x80 | bytes); //enable read | number of byte
		delayMs(10);
	}

	void MPU9250::resetMagnetometer() {
		writeAK8963Register(REGISTER_AK8963_CNTL_2, 0x01);
		delayMs(100);
	}

	Vector3F MPU9250::readMagValues() {
		uint8_t rawData[6];
		readAK8963Data(rawData);

		const auto x = static_cast<int16_t>((rawData[1] << 8) | rawData[0]);
		const auto y = static_cast<int16_t>((rawData[3] << 8) | rawData[2]);
		const auto z = static_cast<int16_t>((rawData[5] << 8) | rawData[4]);

		constexpr static float scaleFactor = 4912.0f / 32760.0f;

		return {
			x * scaleFactor * magCorrFactor.getX(),
			y * scaleFactor * magCorrFactor.getY(),
			z * scaleFactor * magCorrFactor.getZ()
		};
	}

	void MPU9250::getAsaVals() {
		uint8_t rawCorr = 0;
		rawCorr = readAK8963Register8(REGISTER_AK8963_ASAX);
		magCorrFactor.setX((0.5 * (rawCorr - 128) / 128.0) + 1.0);
		rawCorr = readAK8963Register8(REGISTER_AK8963_ASAY);
		magCorrFactor.setY((0.5 * (rawCorr - 128) / 128.0) + 1.0);
		rawCorr = readAK8963Register8(REGISTER_AK8963_ASAZ);
		magCorrFactor.setZ((0.5 * (rawCorr - 128) / 128.0) + 1.0);
	}

	void MPU9250::writeAK8963Register(uint8_t reg, uint8_t val) {
		writeMPU9250Register(REGISTER_I2C_SLV0_ADDR, MAGNETOMETER_I2C_ADDRESS); // write AK8963
		writeMPU9250Register(REGISTER_I2C_SLV0_REG, reg); // define AK8963 register to be written to
		writeMPU9250Register(REGISTER_I2C_SLV0_DO, val);
	}

	uint8_t MPU9250::readAK8963Register8(uint8_t reg) {
		enableMagDataRead(reg, 0x01);
		uint8_t const regVal = readMPU9250Register8(REGISTER_EXT_SLV_SENS_DATA_00);
		enableMagDataRead(REGISTER_AK8963_HXL, 0x08);

		return regVal;
	}

	void MPU9250::readAK8963Data(uint8_t* buf) {
		// Write
		const uint8_t cmd = REGISTER_EXT_SLV_SENS_DATA_00 | 0x80;
		ESP_ERROR_CHECK(i2c_master_transmit(_I2CDeviceHandle, &cmd, 1, -1));

		// Read
		ESP_ERROR_CHECK(i2c_master_receive(_I2CDeviceHandle, buf, 6, -1));

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

	void MPU9250::setMagnetometer16Bit() {
		uint8_t regVal = readAK8963Register8(REGISTER_AK8963_CNTL_1);
		regVal |= REGISTER_VALUE_AK8963_16_BIT;
		writeAK8963Register(REGISTER_AK8963_CNTL_1, regVal);
	}

	uint8_t MPU9250::getStatus2Register() {
		return readAK8963Register8(REGISTER_AK8963_STATUS_2);
	}

	void MPU9250::delayMs(uint32_t ms) {
		vTaskDelay(ms <= portTICK_PERIOD_MS ? portTICK_PERIOD_MS : pdMS_TO_TICKS(ms));
	}
}
