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

#pragma once

#include <cstdint>
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "types/vector3.h"

namespace pizda {
	typedef enum MPU9250_BW_WO_DLPF {
		MPU9250_BW_WO_DLPF_3600 = 0x02,
		MPU9250_BW_WO_DLPF_8800 = 0x01,
	} MPU9250_bw_wo_dlpf;

	typedef enum MPU9250_DLPF {
		MPU9250_DLPF_0, MPU9250_DLPF_1, MPU9250_DLPF_2, MPU9250_DLPF_3, MPU9250_DLPF_4, MPU9250_DLPF_5,
		MPU9250_DLPF_6, MPU9250_DLPF_7,
	} MPU9250_dlpf;

	typedef enum MPU9250_GYRO_RANGE {
		MPU9250_GYRO_RANGE_250, MPU9250_GYRO_RANGE_500, MPU9250_GYRO_RANGE_1000, MPU9250_GYRO_RANGE_2000,
	} MPU9250_gyroRange;

	typedef enum MPU9250_ACC_RANGE {
		MPU9250_ACC_RANGE_2G, MPU9250_ACC_RANGE_4G, MPU9250_ACC_RANGE_8G, MPU9250_ACC_RANGE_16G,
	} MPU9250_accRange;

	typedef enum MPU9250_LOW_PWR_ACC_ODR {
		MPU9250_LP_ACC_ODR_0_24, MPU9250_LP_ACC_ODR_0_49, MPU9250_LP_ACC_ODR_0_98, MPU9250_LP_ACC_ODR_1_95,
		MPU9250_LP_ACC_ODR_3_91, MPU9250_LP_ACC_ODR_7_81, MPU9250_LP_ACC_ODR_15_63, MPU9250_LP_ACC_ODR_31_25,
		MPU9250_LP_ACC_ODR_62_5, MPU9250_LP_ACC_ODR_125, MPU9250_LP_ACC_ODR_250, MPU9250_LP_ACC_ODR_500,
	} MPU9250_lpAccODR;

	typedef enum MPU9250_INT_PIN_POL {
		MPU9250_ACT_HIGH, MPU9250_ACT_LOW,
	} MPU9250_intPinPol;

	typedef enum MPU9250_INT_TYPE {
		MPU9250_DATA_READY = 0x01,
		MPU9250_FIFO_OVF = 0x10,
		MPU9250_WOM_INT = 0x40,
	} MPU9250_intType;

	typedef enum MPU9250_WOM_EN {
		MPU9250_WOM_DISABLE, MPU9250_WOM_ENABLE,
	} MPU9250_womEn;

	typedef enum MPU9250_WOM_COMP {
		MPU9250_WOM_COMP_DISABLE, MPU9250_WOM_COMP_ENABLE,
	} MPU9250_womCompEn;

	typedef enum MPU9250_XYZ_ENABLE {
		MPU9250_ENABLE_XYZ,  //all axes are enabled (default)
		MPU9250_ENABLE_XY0,  // x, y enabled, z disabled
		MPU9250_ENABLE_X0Z,
		MPU9250_ENABLE_X00,
		MPU9250_ENABLE_0YZ,
		MPU9250_ENABLE_0Y0,
		MPU9250_ENABLE_00Z,
		MPU9250_ENABLE_000,  // all axes disabled
	} MPU9250_xyzEn;

	typedef enum MPU9250_ORIENTATION {
		MPU9250_FLAT,
		MPU9250_FLAT_1,
		MPU9250_XY,
		MPU9250_XY_1,
		MPU9250_YX,
		MPU9250_YX_1,
	} MPU9250_orientation;

	typedef enum MPU9250_FIFO_MODE {
		MPU9250_CONTINUOUS,
		MPU9250_STOP_WHEN_FULL
	} MPU9250_fifoMode;

	typedef enum MPU9250_FIFO_TYPE {
		MPU9250_FIFO_ACC = 0x08,
		MPU9250_FIFO_GYR = 0x70,
		MPU9250_FIFO_ACC_GYR = 0x78
	} MPU9250_fifo_type;

	typedef enum AK8963_OP_MODE {
		AK8963_PWR_DOWN = 0x00,
		AK8963_TRIGGER_MODE = 0x01,
		AK8963_CONT_MODE_8HZ = 0x02,
		AK8963_CONT_MODE_100HZ = 0x06,
		AK8963_FUSE_ROM_ACC_MODE = 0x0F
	} AK8963_opMode;

	class MPU9250 {
		public:
			/* Registers MPU6500 */
			static uint8_t constexpr REGISTER_SELF_TEST_X_GYRO = 0x00;
			static uint8_t constexpr REGISTER_SELF_TEST_Y_GYRO = 0x01;
			static uint8_t constexpr REGISTER_SELF_TEST_Z_GYRO = 0x02;
			static uint8_t constexpr REGISTER_SELF_TEST_X_ACCEL = 0x0D;
			static uint8_t constexpr REGISTER_SELF_TEST_Y_ACCEL = 0x0E;
			static uint8_t constexpr REGISTER_SELF_TEST_Z_ACCEL = 0x0F;
			static uint8_t constexpr REGISTER_XG_OFFSET_H = 0x13;
			static uint8_t constexpr REGISTER_XG_OFFSET_L = 0x14;
			static uint8_t constexpr REGISTER_YG_OFFSET_H = 0x15;
			static uint8_t constexpr REGISTER_YG_OFFSET_L = 0x16;
			static uint8_t constexpr REGISTER_ZG_OFFSET_H = 0x17;
			static uint8_t constexpr REGISTER_ZG_OFFSET_L = 0x18;
			static uint8_t constexpr REGISTER_SMPLRT_DIV = 0x19;
			static uint8_t constexpr REGISTER_CONFIG = 0x1A;
			static uint8_t constexpr REGISTER_GYRO_CONFIG = 0x1B;
			static uint8_t constexpr REGISTER_ACCEL_CONFIG = 0x1C;
			static uint8_t constexpr REGISTER_ACCEL_CONFIG_2 = 0x1D;
			static uint8_t constexpr REGISTER_LP_ACCEL_ODR = 0x1E;
			static uint8_t constexpr REGISTER_WOM_THR = 0x1F;
			static uint8_t constexpr REGISTER_FIFO_EN = 0x23;
			static uint8_t constexpr REGISTER_I2C_MST_CTRL = 0x24;
			static uint8_t constexpr REGISTER_I2C_SLV0_ADDR = 0x25;
			static uint8_t constexpr REGISTER_I2C_SLV0_REG = 0x26;
			static uint8_t constexpr REGISTER_I2C_SLV0_CTRL = 0x27;
			static uint8_t constexpr REGISTER_I2C_MST_STATUS = 0x36;
			static uint8_t constexpr REGISTER_INT_PIN_CFG = 0x37;
			static uint8_t constexpr REGISTER_INT_ENABLE = 0x38;
			static uint8_t constexpr REGISTER_INT_STATUS = 0x3A;
			static uint8_t constexpr REGISTER_ACCEL_OUT = 0x3B; // accel data registers begin
			static uint8_t constexpr REGISTER_TEMP_OUT = 0x41;
			static uint8_t constexpr REGISTER_GYRO_OUT = 0x43; // gyro data registers begin
			static uint8_t constexpr REGISTER_EXT_SLV_SENS_DATA_00 = 0x49;
			static uint8_t constexpr REGISTER_I2C_SLV0_DO = 0x63;
			static uint8_t constexpr REGISTER_I2C_MST_DELAY_CTRL = 0x67;
			static uint8_t constexpr REGISTER_SIGNAL_PATH_RESET = 0x68;
			static uint8_t constexpr REGISTER_MOT_DET_CTRL = 0x69;
			static uint8_t constexpr REGISTER_USER_CTRL = 0x6A;
			static uint8_t constexpr REGISTER_PWR_MGMT_1 = 0x6B;
			static uint8_t constexpr REGISTER_PWR_MGMT_2 = 0x6C;
			static uint8_t constexpr REGISTER_FIFO_COUNT = 0x72; // 0x72 is COUNT_H
			static uint8_t constexpr REGISTER_FIFO_R_W = 0x74;
			static uint8_t constexpr REGISTER_WHO_AM_I = 0x75;
			static uint8_t constexpr REGISTER_XA_OFFSET_H = 0x77;
			static uint8_t constexpr REGISTER_XA_OFFSET_L = 0x78;
			static uint8_t constexpr REGISTER_YA_OFFSET_H = 0x7A;
			static uint8_t constexpr REGISTER_YA_OFFSET_L = 0x7B;
			static uint8_t constexpr REGISTER_ZA_OFFSET_H = 0x7D;
			static uint8_t constexpr REGISTER_ZA_OFFSET_L = 0x7E;

			/* Register Values */
			static uint8_t constexpr REGISTER_VALUE_RESET = 0x80;
			static uint8_t constexpr REGISTER_VALUE_BYPASS_EN = 0x02;
			static uint8_t constexpr REGISTER_VALUE_I2C_MST_EN = 0x20;
			static uint8_t constexpr REGISTER_VALUE_CLK_SEL_PLL = 0x01;

			/* Others */
			static float constexpr ROOM_TEMPERATURE_OFFSET = 0.0f;
			static float constexpr TEMPERATURE_SENSITIVITY = 333.87f;

			/* Register Values */
			static uint8_t constexpr REGISTER_VALUE_AK8963_16_BIT = 0x10;
			static uint8_t constexpr REGISTER_VALUE_AK8963_OVF = 0x08;
			static uint8_t constexpr REGISTER_VALUE_AK8963_READ = 0x80;

			/* Others */
			static uint8_t constexpr WHO_AM_I_CODE = 0x71;
			static uint8_t constexpr MAGNETOMETER_I2C_ADDRESS = 0x0C;
			static uint8_t constexpr MAGNETOMETER_WHO_AM_I_CODE = 0x48;

			/* Registers AK8963 */
			static uint8_t constexpr REGISTER_AK8963_WIA = 0x00; // Who am I
			static uint8_t constexpr REGISTER_AK8963_INFO = 0x01;
			static uint8_t constexpr REGISTER_AK8963_STATUS_1 = 0x02;
			static uint8_t constexpr REGISTER_AK8963_HXL = 0x03;
			static uint8_t constexpr REGISTER_AK8963_HYL = 0x05;
			static uint8_t constexpr REGISTER_AK8963_HZL = 0x07;
			static uint8_t constexpr REGISTER_AK8963_STATUS_2 = 0x09;
			static uint8_t constexpr REGISTER_AK8963_CNTL_1 = 0x0A;
			static uint8_t constexpr REGISTER_AK8963_CNTL_2 = 0x0B;
			static uint8_t constexpr REGISTER_AK8963_ASTC = 0x0C; // Self Test
			static uint8_t constexpr REGISTER_AK8963_I2CDIS = 0x0F;
			static uint8_t constexpr REGISTER_AK8963_ASAX = 0x10;
			static uint8_t constexpr REGISTER_AK8963_ASAY = 0x11;
			static uint8_t constexpr REGISTER_AK8963_ASAZ = 0x12;

			bool setup(i2c_master_bus_handle_t I2CBusHandle, uint8_t I2CAddress);

			uint8_t readWhoAmI();

			/* The slope of the curve of acceleration vs measured values fits quite well to the theoretical
			   * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the
			   * MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset
			   * values. It assumes your MPU9250 is positioned flat with its x,y-plane. The more you deviate
			   * from this, the less accurate will be your results.
			   * The function also measures the offset of the gyroscope data. The gyroscope offset does not
			   * depend on the positioning.
			   * This function needs to be called at the beginning since it can overwrite your settings!
			   */
			void calibrateAccAndGyr();

			/*  This is a more accurate method for calibration. You have to determine the minimum and maximum
			 *  raw acceleration values of the axes determined in the range +/- 2 g.
			 *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
			 *  Use either autoOffset or setAccOffsets, not both.
			 */
			void setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);

			void setAccOffsets(Vector3F offset); // for writing back previous offsets

			/*  The gyroscope data is not zero, even if you don't move the MPU9250.
			 *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
			 *  using the +/- 250 degrees/s range.
			 *  Use either autoOffset or setGyrOffsets, not both.
			 */
			void setGyrOffsets(Vector3F offset); // for writing back previous offsets
			Vector3F getAccOffsets();

			Vector3F getGyrOffsets();

			/*  Digital Low Pass Filter for the gyroscope must be enabled to choose the level.
			 *  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7
			 *
			 *  DLPF    Bandwidth [Hz]   Delay [ms]   Output Rate [kHz]
			 *    0         250            0.97             8
			 *    1         184            2.9              1
			 *    2          92            3.9              1
			 *    3          41            5.9              1
			 *    4          20            9.9              1
			 *    5          10           17.85             1
			 *    6           5           33.48             1
			 *    7        3600            0.17             8
			 *
			 *    You achieve lowest noise using level 6
			 */
			void setGyrDLPF(MPU9250_dlpf dlpf);

			/*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
			 *  Sample rate = Internal sample rate / (1 + divider)
			 *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
			 *  Divider is a number 0...255
			 */
			void setSampleRateDivider(uint8_t splRateDiv);

			void setGyrRange(MPU9250_gyroRange gyroRange);

			/*  You can enable or disable the digital low pass filter (DLPF). If you disable the DLPF, you
		  *  need to select the bandwidth, which can be either 8800 or 3600 Hz. 8800 Hz has a shorter delay,
		  *  but higher noise level. If DLPF is disabled, the output rate is 32 kHz.
		  *  MPU9250_BW_WO_DLPF_3600
		  *  MPU9250_BW_WO_DLPF_8800
		  */
			void enableGyrDLPF();

			void disableGyrDLPF(MPU9250_bw_wo_dlpf bw);

			void setAccRange(MPU9250_accRange accRange);

			/* Enable/disable the digital low pass filter for the accelerometer
			*  If disabled the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
			*/
			void enableAccDLPF(bool enable);

			/*  Digital low pass filter (DLPF) for the accelerometer (if DLPF enabled)
			*  MPU9250_DPLF_0, MPU9250_DPLF_2, ...... MPU9250_DPLF_7
			*   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
			*     0           460               1.94           1
			*     1           184               5.80           1
			*     2            92               7.80           1
			*     3            41              11.80           1
			*     4            20              19.80           1
			*     5            10              35.70           1
			*     6             5              66.96           1
			*     7           460               1.94           1
			*/
			void setAccDLPF(MPU9250_dlpf dlpf);

			void setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr);

			void enableAccAxes(MPU9250_xyzEn enable);

			void enableGyrAxes(MPU9250_xyzEn enable);

			/* x,y,z results */

			Vector3F getAccRawValues();

			Vector3F getCorrectedAccRawValues();

			Vector3F getGValues();

			Vector3F getAccRawValuesFromFifo();

			Vector3F getCorrectedAccRawValuesFromFifo();

			Vector3F getGValuesFromFifo();

			float getResultantG(Vector3F gVal);

			float readTemperature();

			Vector3F readGyroRawValues();

			Vector3F readCorrectedGyroRawValues();

			Vector3F readGyroValues();

			Vector3F readGyroValuesFromFifo();


			/* Angles and Orientation */

			Vector3F getAngles();

			MPU9250_orientation getOrientation();

			float getPitch();

			float getRoll();

			/* Power, Sleep, Standby */

			void sleep(bool sleep);

			void enableCycle(bool cycle);

			void enableGyrStandby(bool gyroStandby);

			/* Interrupts */

			void setIntPinPolarity(MPU9250_intPinPol pol);

			void enableIntLatch(bool latch);

			void enableClearIntByAnyRead(bool clearByAnyRead);

			void enableInterrupt(MPU9250_intType intType);

			void disableInterrupt(MPU9250_intType intType);

			bool checkInterrupt(uint8_t source, MPU9250_intType type);

			uint8_t readAndClearInterrupts();

			void setWakeOnMotionThreshold(uint8_t womthresh);

			void enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn);

			/* FIFO */

			void startFifo(MPU9250_fifo_type fifo);

			void stopFifo();

			void enableFifo(bool fifo);

			void resetFifo();

			int16_t getFifoCount();

			void setFifoMode(MPU9250_fifoMode mode);

			int16_t getNumberOfFifoDataSets();

			void findFifoBegin();

			/* x,y,z results */

			Vector3F getMagValues();

			/* Magnetometer */

			bool initMagnetometer();

			uint8_t readWhoAmIMag();

			void setMagOpMode(AK8963_opMode opMode);

			void startMagMeasurement();

		private:
			Vector3F accOffsetVal;
			Vector3F gyrOffsetVal;
			uint8_t accRangeFactor;
			uint8_t gyrRangeFactor;
			MPU9250_fifo_type fifoType;
			Vector3F magCorrFactor;

			i2c_master_dev_handle_t _I2CDeviceHandle{};

			void delayMs(uint32_t ms) {
				vTaskDelay(ms <= portTICK_PERIOD_MS ? portTICK_PERIOD_MS : pdMS_TO_TICKS(ms));
			}

			void correctAccRawValues(Vector3F& rawValues);

			void correctGyrRawValues(Vector3F& rawValues);

			void getAsaVals();

			void reset_MPU9250();

			void enableI2CMaster();

			void writeMPU9250Register(uint8_t reg, uint8_t val);

			uint8_t readMPU9250Register8(uint8_t reg);

			int16_t readMPU9250Register16(uint8_t reg);

			void readMPU9250Register3x16(uint8_t reg, uint8_t* buf);

			Vector3F readMPU9250xyzValFromFifo();

			void enableMagDataRead(uint8_t reg, uint8_t bytes);

			void resetMagnetometer();

			void writeAK8963Register(uint8_t reg, uint8_t val);

			uint8_t readAK8963Register8(uint8_t reg);

			void readAK8963Data(uint8_t* buf);

			void setMagnetometer16Bit();

			uint8_t getStatus2Register();

			uint8_t const i2cAddress = 0x68;
	};
}