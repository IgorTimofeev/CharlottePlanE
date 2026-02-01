#pragma once

#include <array>

#include <NVSSettings.h>
#include <NVSStream.h>
#include <vector3.h>

#include "config.h"

namespace pizda {
	using namespace YOBA;
	
	class ADIRSSettingsUnit {
		public:
			Vector3F accelBias {};
			Vector3F gyroBias {};
			Vector3F magBias {};
	};
	
	class ADIRSSettings : public NVSSettings {
		public:
			std::array<ADIRSSettingsUnit, config::adirs::unitsQuantity> units {};
			uint32_t referencePressurePa;

		protected:
			const char* getNamespace() override {
				return "adirs1";
			}

			void onRead(const NVSStream& stream) override {
				referencePressurePa = stream.readUint32(_referencePressurePa, 101325);

				// Units
				{
					const auto readUnitsQuantity = stream.readObjectLength<ADIRSSettingsUnit>(_units);

					if (readUnitsQuantity <= 0)
						return;

					if (readUnitsQuantity != config::adirs::unitsQuantity) {
						ESP_LOGI("ADIRSSettings", "read units length (%d) != config length (%d)", readUnitsQuantity, config::adirs::unitsQuantity);
						return;
					}

					stream.readObject<ADIRSSettingsUnit>(_units, units.data(), readUnitsQuantity);
				}
			}

			void onWrite(const NVSStream& stream) override {
				stream.writeUint32(_referencePressurePa, referencePressurePa);
				stream.writeObject<ADIRSSettingsUnit>(_units, units.data(), config::adirs::unitsQuantity);
			}
			
		private:
			constexpr static auto _units = "un";
			constexpr static auto _referencePressurePa = "rp";
	};
}