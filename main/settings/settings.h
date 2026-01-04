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

		protected:
			const char* getNamespace() override {
				return "rmtr1";
			}

			void onRead(const NVSStream& stream) override {
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

			void onWrite(const NVSStream& stream) override {
				if (configurations.empty()) {
					stream.erase(_configurations);
				}
				else {
					stream.writeObject<MotorConfiguration>(_configurations, configurations.data(), configurations.size());
				}
			}

			private:
				constexpr static auto _configurations = "cnf";
	};
	
	class Settings {
		public:
			MotorSettings motors {};

			void setup() {
				motors.read();
			}
	};
}