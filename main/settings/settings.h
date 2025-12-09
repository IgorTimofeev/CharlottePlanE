#pragma once

#include <vector>

#include <YOBANVS/main.h>

#include "hardware/motor.h"
#include "hardware/transceiver/channels.h"

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
				const auto readSize = stream.getObjectLength<MotorConfiguration>(_configurations);

				configurations.clear();

				if (readSize > 0) {
					configurations.resize(readSize);
					stream.getObject<MotorConfiguration>(_configurations, configurations.data(), readSize);

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
					stream.setObject<MotorConfiguration>(_configurations, configurations.data(), configurations.size());
				}
			}

			private:
				constexpr static auto _configurations = "cnf";
		};

	#pragma pack(push, 1)
	class ChannelDataStructureSettingsField {
		public:
			ChannelDataType type = ChannelDataType::unsignedInteger;
			uint8_t bitDepth = 8;
			uint8_t count = 1;
	};
	#pragma pack(pop)

	class ChannelDataStructureSettings : public NVSSettings {
		public:
			std::vector<ChannelDataStructureSettingsField> fields {};

			size_t getRequiredBitCountForChannels() {
				size_t result = 0;

				for (auto& field : fields)
					result += field.bitDepth * field.count;

				return result;
			}

		protected:
			const char* getNamespace() override {
				return "rcds1";
			}

			void onRead(const NVSStream& stream) override {
				const auto readSize = stream.getObjectLength<ChannelDataStructureSettingsField>(_fields);

				fields.clear();

				if (readSize > 0) {
					fields.resize(readSize);
					stream.getObject<ChannelDataStructureSettingsField>(_fields, fields.data(), readSize);
				}
			}

			void onWrite(const NVSStream& stream) override {
				if (fields.empty()) {
					stream.erase(_fields);
				}
				else {
					stream.setObject<ChannelDataStructureSettingsField>(_fields, fields.data(), fields.size());
				}
			}

		private:
			constexpr static auto _fields = "fld";

	};

	class Settings {
		public:
			ChannelDataStructureSettings channelDataStructure {};
			MotorSettings motors {};

			void setup() {
				NVSSettings::setup();

				channelDataStructure.read();
				motors.read();
			}
	};
}