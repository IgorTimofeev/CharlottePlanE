#pragma once

#include <cstdint>

#include <NVSSettings.h>

#include "types/generic.h"

namespace pizda {
	using namespace YOBA;
	
	class MotorSettings : public NVSSettings {
		public:
			MotorConfiguration throttle {};
			MotorConfiguration noseWheel {};
			
			MotorConfiguration aileronLeft {};
			MotorConfiguration aileronRight {};
			
			MotorConfiguration flapRight {};
			MotorConfiguration flapLeft {};
			
			MotorConfiguration tailLeft {};
			MotorConfiguration tailRight {};
		
		protected:
			const char* getNamespace() override {
				return "mt1";
			}
			
			void onRead(const NVSStream& stream) override {
				readMotor(
					stream,
					throttle,
					
					_throttleMin,
					_throttleMax,
					_throttleStartup,
					_throttleOffset,
					_throttleReverse
				);
				
				readMotor(
					stream,
					noseWheel,
					
					_noseWheelMin,
					_noseWheelMax,
					_noseWheelStartup,
					_noseWheelOffset,
					_noseWheelReverse
				);
				
				readMotor(
					stream,
					aileronLeft,
					
					_aileronLeftMin,
					_aileronLeftMax,
					_aileronLeftStartup,
					_aileronLeftOffset,
					_aileronLeftReverse
				);
				
				readMotor(
					stream,
					aileronRight,
					
					_aileronRightMin,
					_aileronRightMax,
					_aileronRightStartup,
					_aileronRightOffset,
					_aileronRightReverse
				);
				
				readMotor(
					stream,
					flapLeft,
					
					_flapLeftMin,
					_flapLeftMax,
					_flapLeftStartup,
					_flapLeftOffset,
					_flapLeftReverse
				);
				
				readMotor(
					stream,
					flapRight,
					
					_flapRightMin,
					_flapRightMax,
					_flapRightStartup,
					_flapRightOffset,
					_flapRightReverse
				);
				
				readMotor(
					stream,
					tailLeft,
					
					_tailLeftMin,
					_tailLeftMax,
					_tailLeftStartup,
					_tailLeftOffset,
					_tailLeftReverse
				);
				
				readMotor(
					stream,
					tailRight,
					
					_tailRightMin,
					_tailRightMax,
					_tailRightStartup,
					_tailRightOffset,
					_tailRightReverse
				);
			}
			
			void onWrite(const NVSStream& stream) override {
				writeMotor(
					stream,
					throttle,
					
					_throttleMin,
					_throttleMax,
					_throttleStartup,
					_throttleOffset,
					_throttleReverse
				);
				
				writeMotor(
					stream,
					noseWheel,
					
					_noseWheelMin,
					_noseWheelMax,
					_noseWheelStartup,
					_noseWheelOffset,
					_noseWheelReverse
				);
				
				writeMotor(
					stream,
					aileronLeft,
					
					_aileronLeftMin,
					_aileronLeftMax,
					_aileronLeftStartup,
					_aileronLeftOffset,
					_aileronLeftReverse
				);
				
				writeMotor(
					stream,
					aileronRight,
					
					_aileronRightMin,
					_aileronRightMax,
					_aileronRightStartup,
					_aileronRightOffset,
					_aileronRightReverse
				);
				
				writeMotor(
					stream,
					flapLeft,
					
					_flapLeftMin,
					_flapLeftMax,
					_flapLeftStartup,
					_flapLeftOffset,
					_flapLeftReverse
				);
				
				writeMotor(
					stream,
					flapRight,
					
					_flapRightMin,
					_flapRightMax,
					_flapRightStartup,
					_flapRightOffset,
					_flapRightReverse
				);
				
				writeMotor(
					stream,
					tailLeft,
					
					_tailLeftMin,
					_tailLeftMax,
					_tailLeftStartup,
					_tailLeftOffset,
					_tailLeftReverse
				);
				
				writeMotor(
					stream,
					tailRight,
					
					_tailRightMin,
					_tailRightMax,
					_tailRightStartup,
					_tailRightOffset,
					_tailRightReverse
				);
			}
		
		private:
			constexpr static const char* _throttleMin = "thm";
			constexpr static const char* _throttleMax = "thx";
			constexpr static const char* _throttleStartup = "ths";
			constexpr static const char* _throttleOffset = "tho";
			constexpr static const char* _throttleReverse = "thr";
			
			constexpr static const char* _noseWheelMin = "nwm";
			constexpr static const char* _noseWheelMax = "nwx";
			constexpr static const char* _noseWheelStartup = "nws";
			constexpr static const char* _noseWheelOffset = "nwo";
			constexpr static const char* _noseWheelReverse = "nwr";
			
			constexpr static const char* _aileronLeftMin = "alm";
			constexpr static const char* _aileronLeftMax = "alx";
			constexpr static const char* _aileronLeftStartup = "als";
			constexpr static const char* _aileronLeftOffset = "alo";
			constexpr static const char* _aileronLeftReverse = "alr";
			
			constexpr static const char* _aileronRightMin = "arm";
			constexpr static const char* _aileronRightMax = "arx";
			constexpr static const char* _aileronRightStartup = "ars";
			constexpr static const char* _aileronRightOffset = "aro";
			constexpr static const char* _aileronRightReverse = "arr";
			
			constexpr static const char* _flapLeftMin = "flm";
			constexpr static const char* _flapLeftMax = "flx";
			constexpr static const char* _flapLeftStartup = "fls";
			constexpr static const char* _flapLeftOffset = "flo";
			constexpr static const char* _flapLeftReverse = "flr";
			
			constexpr static const char* _flapRightMin = "frm";
			constexpr static const char* _flapRightMax = "frx";
			constexpr static const char* _flapRightStartup = "frs";
			constexpr static const char* _flapRightOffset = "fro";
			constexpr static const char* _flapRightReverse = "frr";
			
			constexpr static const char* _tailLeftMin = "tlm";
			constexpr static const char* _tailLeftMax = "tlx";
			constexpr static const char* _tailLeftStartup = "tls";
			constexpr static const char* _tailLeftOffset = "tlo";
			constexpr static const char* _tailLeftReverse = "tlr";
			
			constexpr static const char* _tailRightMin = "trm";
			constexpr static const char* _tailRightMax = "trx";
			constexpr static const char* _tailRightStartup = "trs";
			constexpr static const char* _tailRightOffset = "tro";
			constexpr static const char* _tailRightReverse = "trr";
			
			static void readMotor(
				const NVSStream& stream,
				MotorConfiguration& motor,
				
				const char* minKey,
				const char* maxKey,
				const char* startupKey,
				const char* offsetKey,
				const char* reverseKey
			) {
				motor.min = stream.readUint16(minKey, 1000);
				motor.max = stream.readUint16(maxKey, 2000);
				motor.startup = stream.readUint16(startupKey, 1500);
				motor.offset = stream.readInt16(offsetKey, 0);
				motor.reverse = stream.readBool(reverseKey, false);
			}
			
			static void writeMotor(
				const NVSStream& stream,
				const MotorConfiguration& motor,
				
				const char* minKey,
				const char* maxKey,
				const char* startupKey,
				const char* offsetKey,
				const char* reverseKey
			) {
				stream.writeUint16(minKey, motor.min);
				stream.writeUint16(maxKey, motor.max);
				stream.writeUint16(startupKey, motor.startup);
				stream.writeInt16(offsetKey, motor.offset);
				stream.writeBool(reverseKey, motor.reverse);
			}
	};
}