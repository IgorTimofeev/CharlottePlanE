#pragma once

#include <vector>

#include <NVSSettings.h>
#include <NVSStream.h>

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
				return "rcds2";
			}

			void onRead(const NVSStream& stream) override {
				const auto readLength = stream.readObjectLength<ChannelDataStructureSettingsField>(_fields);

				fields.clear();

				if (readLength > 0) {
					fields.resize(readLength);
					stream.readObject<ChannelDataStructureSettingsField>(_fields, fields.data(), readLength);
				}
			}

			void onWrite(const NVSStream& stream) override {
				if (fields.empty()) {
					stream.erase(_fields);
				}
				else {
					stream.writeObject<ChannelDataStructureSettingsField>(_fields, fields.data(), fields.size());
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
				channelDataStructure.read();
				motors.read();
			}
	};
}