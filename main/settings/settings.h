#pragma once

#include <vector>

#include <YOBANVS/main.h>

namespace pizda {
	using namespace YOBA;

	#pragma pack(push, 1)
	class MotorConfiguration {
		public:
			uint16_t min = 1000;
			uint16_t max = 2000;
			uint16_t startup = 1500;
			int16_t offset = 0;
			bool reverse = false;

			void sanitize() {
				min = std::clamp<uint16_t>(min, 100, 1400);
				max = std::clamp<uint16_t>(max, 1600, 2900);

				if (min > max)
					std::swap(min, max);

				startup = std::clamp<uint16_t>(startup, min, max);

				if (std::abs(offset) > 900)
					offset = 0;
			}
	};
	#pragma pack(pop)

	class RemoteMotorSettings : public NVSSettings {
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
					for (auto& item : configurations)
						item.sanitize();

					stream.setObject<MotorConfiguration>(_configurations, configurations.data(), configurations.size());
				}
			}

			private:
				constexpr static auto _configurations = "cnf";
		};

	enum class RemoteChannelDataStructureSettingsChannelType : uint8_t {
		unsignedInteger,
		boolean
	};

	#pragma pack(push, 1)
	class RemoteChannelDataStructureSettingsField {
		public:
			RemoteChannelDataStructureSettingsField() {

			}

			RemoteChannelDataStructureSettingsChannelType type = RemoteChannelDataStructureSettingsChannelType::unsignedInteger;
			uint8_t bitDepth = 8;
			uint8_t count = 1;
	};
	#pragma pack(pop)

	class RemoteChannelDataStructureSettings : public NVSSettings {
		public:
			std::vector<RemoteChannelDataStructureSettingsField> fields {};

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
				const auto readSize = stream.getObjectLength<RemoteChannelDataStructureSettingsField>(_fields);

				fields.clear();

				if (readSize > 0) {
					fields.resize(readSize);
					stream.getObject<RemoteChannelDataStructureSettingsField>(_fields, fields.data(), readSize);
				}
			}

			void onWrite(const NVSStream& stream) override {
				if (fields.empty()) {
					stream.erase(_fields);
				}
				else {
					stream.setObject<RemoteChannelDataStructureSettingsField>(_fields, fields.data(), fields.size());
				}
			}

		private:
			constexpr static auto _fields = "fld";

	};

	class Settings {
		public:
			RemoteChannelDataStructureSettings remoteChannelDataStructure {};
			RemoteMotorSettings motors {};

			void setup() {
				NVSSettings::setup();

				remoteChannelDataStructure.read();
				motors.read();
			}
	};
}