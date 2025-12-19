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
#include "hardware/busStream.h"

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
			bool setup(i2c_master_bus_handle_t I2CBusHandle, uint8_t I2CAddress);

			uint8_t readWhoAmI();

			/*  Digital Low Pass Filter for the gyroscope must be enabled to choose the level.
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
			void setSRD(uint8_t splRateDiv);

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
			void enableAccDLPF();
			void disableAccDLPF();

			/*  Digital low pass filter (DLPF) for the accelerometer (if DLPF enabled)
			*
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

			Vector3F readAccValues();
			Vector3F readAccValuesFromFIFO();

			Vector3F readGyroValues();
			Vector3F readGyroValuesFromFIFO();

			float readTemperature();

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

			/*  If latch is enabled the interrupt pin level is held until the interrupt status
			 *  is cleared. If latch is disabled the interrupt pulse is ~50µs (default).
			 */
			void enableIntLatch(bool latch);

			/*  The interrupt can be cleared by any read or it will only be cleared if the interrupt
			 *  status register is read (default).
			 */
			void enableClearIntByAnyRead(bool clearByAnyRead);

			/*  Enable/disable interrupts:
			 *  MPU9250_DATA_READY
			 *  MPU9250_FIFO_OVF
			 *  MPU9250_WOM_INT
			 *
			 *  You can enable all interrupts.
			 */
			void enableInterrupt(MPU9250_intType intType);

			void disableInterrupt(MPU9250_intType intType);

			bool checkInterrupt(uint8_t source, MPU9250_intType type);

			uint8_t readAndClearInterruptStatus();

			void setWakeOnMotionThreshold(uint8_t womthresh);

			void enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn);

			/* FIFO */

			void startFIFO(MPU9250_fifo_type fifo);

			void stopFIFO();

			void enableFIFO();
			void disableFIFO();

			void resetFIFO();

			int16_t readFIFOCount();

			void setFIFOMode(MPU9250_fifoMode mode);

			int16_t readFIFODataSetsCount();

			/* This is needed for continuous Fifo mode.
			 * The Fifo buffer ends with a complete data set, but the start is within a data set. 512/6 or 512/12
			 *
			 * Remarks: seems like in cont. mode buffer can start from part of dataset, so we should just skip first values
			 * to prevent partial data read
			 * */
			void findFIFOBegin();

			/* x,y,z results */
			Vector3F readMagValues();
			/* Magnetometer */

			bool setupMagnetometer();

			uint8_t readWhoAmIMag();

			void setMagOpMode(AK8963_opMode opMode);

			void startMagMeasurement();

		private:
			constexpr static const char* _logTag = "MPU-9250";

			/* Registers MPU6500 */
			constexpr static uint8_t REGISTER_SELF_TEST_X_GYRO = 0x00;
			constexpr static uint8_t REGISTER_SELF_TEST_Y_GYRO = 0x01;
			constexpr static uint8_t REGISTER_SELF_TEST_Z_GYRO = 0x02;
			constexpr static uint8_t REGISTER_SELF_TEST_X_ACCEL = 0x0D;
			constexpr static uint8_t REGISTER_SELF_TEST_Y_ACCEL = 0x0E;
			constexpr static uint8_t REGISTER_SELF_TEST_Z_ACCEL = 0x0F;
			constexpr static uint8_t REGISTER_XG_OFFSET_H = 0x13;
			constexpr static uint8_t REGISTER_XG_OFFSET_L = 0x14;
			constexpr static uint8_t REGISTER_YG_OFFSET_H = 0x15;
			constexpr static uint8_t REGISTER_YG_OFFSET_L = 0x16;
			constexpr static uint8_t REGISTER_ZG_OFFSET_H = 0x17;
			constexpr static uint8_t REGISTER_ZG_OFFSET_L = 0x18;
			constexpr static uint8_t REGISTER_SMPLRT_DIV = 0x19;
			constexpr static uint8_t REGISTER_CONFIG = 0x1A;
			constexpr static uint8_t REGISTER_GYRO_CONFIG = 0x1B;
			constexpr static uint8_t REGISTER_ACCEL_CONFIG = 0x1C;
			constexpr static uint8_t REGISTER_ACCEL_CONFIG_2 = 0x1D;
			constexpr static uint8_t REGISTER_LP_ACCEL_ODR = 0x1E;
			constexpr static uint8_t REGISTER_WOM_THR = 0x1F;
			constexpr static uint8_t REGISTER_FIFO_EN = 0x23;
			constexpr static uint8_t REGISTER_I2C_MST_CTRL = 0x24;
			constexpr static uint8_t REGISTER_I2C_SLV0_ADDR = 0x25;
			constexpr static uint8_t REGISTER_I2C_SLV0_REG = 0x26;
			constexpr static uint8_t REGISTER_I2C_SLV0_CTRL = 0x27;
			constexpr static uint8_t REGISTER_I2C_MST_STATUS = 0x36;
			constexpr static uint8_t REGISTER_INT_PIN_CFG = 0x37;
			constexpr static uint8_t REGISTER_INT_ENABLE = 0x38;
			constexpr static uint8_t REGISTER_INT_STATUS = 0x3A;
			constexpr static uint8_t REGISTER_ACCEL_OUT = 0x3B; // accel data registers begin
			constexpr static uint8_t REGISTER_TEMP_OUT = 0x41;
			constexpr static uint8_t REGISTER_GYRO_OUT = 0x43; // gyro data registers begin
			constexpr static uint8_t REGISTER_EXT_SLV_SENS_DATA_00 = 0x49;
			constexpr static uint8_t REGISTER_I2C_SLV0_DO = 0x63;
			constexpr static uint8_t REGISTER_I2C_MST_DELAY_CTRL = 0x67;
			constexpr static uint8_t REGISTER_SIGNAL_PATH_RESET = 0x68;
			constexpr static uint8_t REGISTER_MOT_DET_CTRL = 0x69;
			constexpr static uint8_t REGISTER_USER_CTRL = 0x6A;
			constexpr static uint8_t REGISTER_PWR_MGMT_1 = 0x6B;
			constexpr static uint8_t REGISTER_PWR_MGMT_2 = 0x6C;
			constexpr static uint8_t REGISTER_FIFO_COUNT = 0x72; // 0x72 is COUNT_H
			constexpr static uint8_t REGISTER_FIFO_R_W = 0x74;
			constexpr static uint8_t REGISTER_WHO_AM_I = 0x75;
			constexpr static uint8_t REGISTER_XA_OFFSET_H = 0x77;
			constexpr static uint8_t REGISTER_XA_OFFSET_L = 0x78;
			constexpr static uint8_t REGISTER_YA_OFFSET_H = 0x7A;
			constexpr static uint8_t REGISTER_YA_OFFSET_L = 0x7B;
			constexpr static uint8_t REGISTER_ZA_OFFSET_H = 0x7D;
			constexpr static uint8_t REGISTER_ZA_OFFSET_L = 0x7E;

			/* Register Values */
			constexpr static uint8_t REGISTER_VALUE_RESET = 0x80;
			constexpr static uint8_t REGISTER_VALUE_BYPASS_EN = 0x02;
			constexpr static uint8_t REGISTER_VALUE_I2C_MST_EN = 0x20;
			constexpr static uint8_t REGISTER_VALUE_CLK_SEL_PLL = 0x01;

			/* Others */
			static float constexpr ROOM_TEMPERATURE_OFFSET = 0.0f;
			static float constexpr TEMPERATURE_SENSITIVITY = 333.87f;

			/* Register Values */
			constexpr static uint8_t REGISTER_VALUE_AK8963_16_BIT = 0x10;
			constexpr static uint8_t REGISTER_VALUE_AK8963_OVF = 0x08;
			constexpr static uint8_t REGISTER_VALUE_AK8963_READ = 0x80;

			/* Others */
			constexpr static uint8_t WHO_AM_I_CODE = 0x71;
			constexpr static uint8_t MAGNETOMETER_I2C_ADDRESS = 0x0C;
			constexpr static uint8_t MAGNETOMETER_WHO_AM_I_CODE = 0x48;

			/* Registers AK8963 */
			constexpr static uint8_t REGISTER_AK8963_WIA = 0x00; // Who am I
			constexpr static uint8_t REGISTER_AK8963_INFO = 0x01;
			constexpr static uint8_t REGISTER_AK8963_STATUS_1 = 0x02;
			constexpr static uint8_t REGISTER_AK8963_HXL = 0x03;
			constexpr static uint8_t REGISTER_AK8963_HYL = 0x05;
			constexpr static uint8_t REGISTER_AK8963_HZL = 0x07;
			constexpr static uint8_t REGISTER_AK8963_STATUS_2 = 0x09;
			constexpr static uint8_t REGISTER_AK8963_CNTL_1 = 0x0A;
			constexpr static uint8_t REGISTER_AK8963_CNTL_2 = 0x0B;
			constexpr static uint8_t REGISTER_AK8963_ASTC = 0x0C; // Self Test
			constexpr static uint8_t REGISTER_AK8963_I2CDIS = 0x0F;
			constexpr static uint8_t REGISTER_AK8963_ASAX = 0x10;
			constexpr static uint8_t REGISTER_AK8963_ASAY = 0x11;
			constexpr static uint8_t REGISTER_AK8963_ASAZ = 0x12;

			float accRangeFactor = 1;
			float gyrRangeFactor = 1;
			Vector3F magCorrFactor {1, 1, 1};
			MPU9250_fifo_type fifoType = MPU9250_FIFO_ACC;

			i2c_master_dev_handle_t _I2CDeviceHandle{};

			void delayMs(uint32_t ms);

			void getAsaVals();

			void reset_MPU9250();

			void enableI2CMaster();

			void writeMPU9250Register(uint8_t reg, uint8_t val);

			uint8_t readMPU9250Register8(uint8_t reg);

			int16_t readMPU9250Register16(uint8_t reg);

			void readMPU9250Register3x16(uint8_t reg, uint8_t* buf);

			Vector3F readVector3ValueFromFIFO();

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