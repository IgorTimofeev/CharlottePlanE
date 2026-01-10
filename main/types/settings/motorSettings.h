#pragma once

#include <vector>

#include <NVSSettings.h>
#include <NVSStream.h>

#include "hardware/motor.h"

namespace pizda {
	using namespace YOBA;
	
	class MotorSettings : public NVSSettings {
		public:
			std::vector<MotorConfiguration> configurations {};
			
			// Pre-mapped to motorPowerMaxValue / 2
			int16_t aileronsTrim = 0;
			int16_t elevatorTrim = 0;
			int16_t rudderTrim = 0;
			
		protected:
			const char* getNamespace() override {
				return "rmtr1";
			}

			void onRead(const NVSStream& stream) override {
				// Configurations
				{
					const auto readSize = stream.readObjectLength<MotorConfiguration>(_configurations);
					
					configurations.clear();
					
					if (readSize > 0) {
						configurations.resize(readSize);
						stream.readObject<MotorConfiguration>(_configurations, configurations.data(), readSize);
						
						for (auto& configuration : configurations) {
							configuration.sanitize();
						}
					}
				}
				
				// Trim
				aileronsTrim = stream.readInt16(_aileronsTrim, 0);
				elevatorTrim = stream.readInt16(_elevatorTrim, 0);
				rudderTrim = stream.readInt16(_rudderTrim, 0);
			}

			void onWrite(const NVSStream& stream) override {
				// Configurations
				{
					if (configurations.empty()) {
						stream.erase(_configurations);
					}
					else {
						stream.writeObject<MotorConfiguration>(_configurations, configurations.data(), configurations.size());
					}
				}
				
				// Trim
				stream.writeInt16(_aileronsTrim, aileronsTrim);
				stream.writeInt16(_elevatorTrim, elevatorTrim);
				stream.writeInt16(_rudderTrim, rudderTrim);
			}

			private:
				constexpr static auto _configurations = "cn";
			
				constexpr static auto _aileronsTrim = "ta";
				constexpr static auto _elevatorTrim = "te";
				constexpr static auto _rudderTrim = "tr";
	};
}