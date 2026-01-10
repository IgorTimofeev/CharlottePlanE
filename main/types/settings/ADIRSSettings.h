#pragma once

#include <vector>

#include <NVSSettings.h>
#include <NVSStream.h>

#include "types/vector3.h"

namespace pizda {
	using namespace YOBA;
	
	class ADIRSSettingsUnit {
		public:
			Vector3F accelBias {};
			Vector3F gyroBias {};
			Vector3F magBias {};
			
			void read(
				const NVSStream& stream,
				
				const char* accelBiasXKey,
				const char* accelBiasYKey,
				const char* accelBiasZKey,
				
				const char* gyroBiasXKey,
				const char* gyroBiasYKey,
				const char* gyroBiasZKey,
				
				const char* magBiasXKey,
				const char* magBiasYKey,
				const char* magBiasZKey
			) {
				readVec(
					stream,
					accelBiasXKey,
					accelBiasYKey,
					accelBiasZKey,
					accelBias
				);
				
				readVec(
					stream,
					gyroBiasXKey,
					gyroBiasYKey,
					gyroBiasZKey,
					gyroBias
				);
				
				readVec(
					stream,
					magBiasXKey,
					magBiasYKey,
					magBiasZKey,
					magBias
				);
			}
			
			void write(
				const NVSStream& stream,
				
				const char* accelBiasXKey,
				const char* accelBiasYKey,
				const char* accelBiasZKey,
				
				const char* gyroBiasXKey,
				const char* gyroBiasYKey,
				const char* gyroBiasZKey,
				
				const char* magBiasXKey,
				const char* magBiasYKey,
				const char* magBiasZKey
			) {
				writeVec(
					stream,
					accelBiasXKey,
					accelBiasYKey,
					accelBiasZKey,
					accelBias
				);
				
				writeVec(
					stream,
					gyroBiasXKey,
					gyroBiasYKey,
					gyroBiasZKey,
					gyroBias
				);
				
				writeVec(
					stream,
					magBiasXKey,
					magBiasYKey,
					magBiasZKey,
					magBias
				);
			}
			
		private:
			static void readVec(const NVSStream& stream, const char* keyX, const char* keyY, const char* keyZ, Vector3F& vec) {
				vec.setX(stream.readFloat(keyX, 0));
				vec.setY(stream.readFloat(keyY, 0));
				vec.setZ(stream.readFloat(keyZ, 0));
			}
			
			static void writeVec(const NVSStream& stream, const char* keyX, const char* keyY, const char* keyZ, const Vector3F& vec) {
				stream.writeFloat(keyX, vec.getX());
				stream.writeFloat(keyY, vec.getY());
				stream.writeFloat(keyZ, vec.getZ());
			}
	};
	
	class ADIRSSettings : public NVSSettings {
		public:
			ADIRSSettingsUnit unit0 {};
			
		protected:
			const char* getNamespace() override {
				return "adirs0";
			}

			void onRead(const NVSStream& stream) override {
				unit0.read(
					stream,
					
					_adirs0AccelBiasX,
					_adirs0AccelBiasY,
					_adirs0AccelBiasZ,
					
					_adirs0GyroBiasX,
					_adirs0GyroBiasY,
					_adirs0GyroBiasZ,
					
					_adirs0MagBiasX,
					_adirs0MagBiasY,
					_adirs0MagBiasZ
				);
			}

			void onWrite(const NVSStream& stream) override {
				unit0.write(
					stream,
					
					_adirs0AccelBiasX,
					_adirs0AccelBiasY,
					_adirs0AccelBiasZ,
					
					_adirs0GyroBiasX,
					_adirs0GyroBiasY,
					_adirs0GyroBiasZ,
					
					_adirs0MagBiasX,
					_adirs0MagBiasY,
					_adirs0MagBiasZ
				);
			}
			
		private:
			constexpr static auto _adirs0AccelBiasX = "a0abx";
			constexpr static auto _adirs0AccelBiasY = "a0aby";
			constexpr static auto _adirs0AccelBiasZ = "a0abz";
			
			constexpr static auto _adirs0GyroBiasX = "a0gbx";
			constexpr static auto _adirs0GyroBiasY = "a0gby";
			constexpr static auto _adirs0GyroBiasZ = "a0gbz";
			
			constexpr static auto _adirs0MagBiasX = "a0mbx";
			constexpr static auto _adirs0MagBiasY = "a0mby";
			constexpr static auto _adirs0MagBiasZ = "a0mbz";
	};
}