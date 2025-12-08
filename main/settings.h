#pragma once

#include <vector>

#include <YOBANVS/main.h>

namespace pizda {
	using namespace YOBA;

	#pragma pack(push, 1)
	class MotorSettings {
		public:
			uint16_t min = 0;
			uint16_t max = 0;
			int16_t offset = 0;
			bool reverse = false;

			void sanitize() {
				min = std::clamp<uint16_t>(min, 100, 1400);
				max = std::clamp<uint16_t>(max, 1600, 2900);

				if (min > max)
					std::swap(min, max);

				if (std::abs(offset) > 900)
					offset = 0;
			}
	};
	#pragma pack(pop)

	class MotorsSettings : public NVSSettings {
		public:
			MotorSettings leftThrottle {};
			MotorSettings rightThrottle {};
			
			MotorSettings leftAileron {};
			MotorSettings rightAileron {};

			MotorSettings leftTail {};
			MotorSettings rightTail {};

			MotorSettings leftFlap {};
			MotorSettings rightFlap {};

		protected:
			const char* getNamespace() override {
				return "scc5";
			}

			void onRead(const NVSStream& stream) override {
				// Throttles
				leftThrottle.min = stream.getUint16(_leftThrottleMin, 1000);
				leftThrottle.max = stream.getUint16(_leftThrottleMax, 2000);
				leftThrottle.offset = stream.getInt16(_leftThrottleOffset, 0);
				leftThrottle.reverse = stream.getBool(_leftThrottleReverse, false);
				leftThrottle.sanitize();

				rightThrottle.min = stream.getUint16(_rightThrottleMin, 1000);
				rightThrottle.max = stream.getUint16(_rightThrottleMax, 2000);
				rightThrottle.offset = stream.getInt16(_rightThrottleOffset, 0);
				rightThrottle.reverse = stream.getBool(_rightThrottleReverse, false);
				rightThrottle.sanitize();

				// Ailerons
				leftAileron.min = stream.getUint16(_leftAileronMin, 1000);
				leftAileron.max = stream.getUint16(_leftAileronMax, 2000);
				leftAileron.offset = stream.getInt16(_leftAileronOffset, 0);
				leftAileron.reverse = stream.getBool(_leftAileronReverse, false);
				leftAileron.sanitize();

				rightAileron.min = stream.getUint16(_rightAileronMin, 1000);
				rightAileron.max = stream.getUint16(_rightAileronMax, 2000);
				rightAileron.offset = stream.getInt16(_rightAileronOffset, 0);
				rightAileron.reverse = stream.getBool(_rightAileronReverse, false);
				rightAileron.sanitize();

				// Tail
				leftFlap.min = stream.getUint16(_leftFlapMin, 1000);
				leftFlap.max = stream.getUint16(_leftFlapMax, 2000);
				leftFlap.offset = stream.getInt16(_leftFlapOffset, 0);
				leftFlap.reverse = stream.getBool(_leftFlapReverse, false);
				leftFlap.sanitize();

				rightFlap.min = stream.getUint16(_rightFlapMin, 1000);
				rightFlap.max = stream.getUint16(_rightFlapMax, 2000);
				rightFlap.offset = stream.getInt16(_rightFlapOffset, 0);
				rightFlap.reverse = stream.getBool(_rightFlapReverse, false);
				rightFlap.sanitize();

				// Flaps
				leftTail.min = stream.getUint16(_leftFlapMin, 1000);
				leftTail.max = stream.getUint16(_leftFlapMax, 2000);
				leftTail.offset = stream.getInt16(_leftFlapOffset, 0);
				leftTail.reverse = stream.getBool(_leftFlapReverse, false);
				leftTail.sanitize();

				rightTail.min = stream.getUint16(_rightTailMin, 1000);
				rightTail.max = stream.getUint16(_rightTailMax, 2000);
				rightTail.offset = stream.getInt16(_rightTailOffset, 0);
				rightTail.reverse = stream.getBool(_rightTailReverse, false);
				rightTail.sanitize();
			}

			void onWrite(const NVSStream& stream) override {
				// Throttles
				stream.setUint16(_leftThrottleMin, leftThrottle.min);
				stream.setUint16(_leftThrottleMax, leftThrottle.max);
				stream.setInt16(_leftThrottleOffset, leftThrottle.offset);
				stream.setBool(_leftThrottleReverse, leftThrottle.reverse);
				leftThrottle.sanitize();

				stream.setUint16(_rightThrottleMin, rightThrottle.min);
				stream.setUint16(_rightThrottleMax, rightThrottle.max);
				stream.setInt16(_rightThrottleOffset, rightThrottle.offset);
				stream.setBool(_rightThrottleReverse, rightThrottle.reverse);
				rightThrottle.sanitize();

				// Ailerons
				stream.setUint16(_leftAileronMin, leftAileron.min);
				stream.setUint16(_leftAileronMax, leftAileron.max);
				stream.setInt16(_leftAileronOffset, leftAileron.offset);
				stream.setBool(_leftAileronReverse, leftAileron.reverse);
				leftAileron.sanitize();

				stream.setUint16(_rightAileronMin, rightAileron.min);
				stream.setUint16(_rightAileronMax, rightAileron.max);
				stream.setInt16(_rightAileronOffset, rightAileron.offset);
				stream.setBool(_rightAileronReverse, rightAileron.reverse);
				rightAileron.sanitize();

				// Tail
				stream.setUint16(_leftTailMin, leftTail.min);
				stream.setUint16(_leftTailMax, leftTail.max);
				stream.setInt16(_leftTailOffset, leftTail.offset);
				stream.setBool(_leftTailReverse, leftTail.reverse);
				leftTail.sanitize();

				stream.setUint16(_rightTailMin, rightTail.min);
				stream.setUint16(_rightTailMax, rightTail.max);
				stream.setInt16(_rightTailOffset, rightTail.offset);
				stream.setBool(_rightTailReverse, rightTail.reverse);
				rightTail.sanitize();

				// Flaps
				stream.setUint16(_leftFlapMin, leftFlap.min);
				stream.setUint16(_leftFlapMax, leftFlap.max);
				stream.setInt16(_leftFlapOffset, leftFlap.offset);
				stream.setBool(_leftFlapReverse, leftFlap.reverse);
				leftFlap.sanitize();

				stream.setUint16(_rightFlapMin, rightFlap.min);
				stream.setUint16(_rightFlapMax, rightFlap.max);
				stream.setInt16(_rightFlapOffset, rightFlap.offset);
				stream.setBool(_rightFlapReverse, rightFlap.reverse);
				rightFlap.sanitize();
			}

			private:
				constexpr static auto _leftThrottleMin = "ltm";
				constexpr static auto _leftThrottleMax = "ltx";
				constexpr static auto _leftThrottleOffset = "lto";
				constexpr static auto _leftThrottleReverse = "ltr";

				constexpr static auto _rightThrottleMin = "rtm";
				constexpr static auto _rightThrottleMax = "rtx";
				constexpr static auto _rightThrottleOffset = "rto";
				constexpr static auto _rightThrottleReverse = "rlr";

				constexpr static auto _leftAileronMin = "lam";
				constexpr static auto _leftAileronMax = "lax";
				constexpr static auto _leftAileronOffset = "lao";
				constexpr static auto _leftAileronReverse = "lar";

				constexpr static auto _rightAileronMin = "ram";
				constexpr static auto _rightAileronMax = "rax";
				constexpr static auto _rightAileronOffset = "rao";
				constexpr static auto _rightAileronReverse = "rar";

				constexpr static auto _leftTailMin = "llm";
				constexpr static auto _leftTailMax = "llx";
				constexpr static auto _leftTailOffset = "llo";
				constexpr static auto _leftTailReverse = "llr";

				constexpr static auto _rightTailMin = "rlm";
				constexpr static auto _rightTailMax = "rlx";
				constexpr static auto _rightTailOffset = "rlo";
				constexpr static auto _rightTailReverse = "rlr";

				constexpr static auto _leftFlapMin = "lfm";
				constexpr static auto _leftFlapMax = "lfx";
				constexpr static auto _leftFlapOffset = "lfo";
				constexpr static auto _leftFlapReverse = "lfr";

				constexpr static auto _rightFlapMin = "rfm";
				constexpr static auto _rightFlapMax = "rfx";
				constexpr static auto _rightFlapOffset = "rfo";
				constexpr static auto _rightFlapReverse = "rfr";
		};

	enum class RemoteChannelDataStructureSettingsChannelType : uint8_t {
		Int,
		Uint,
		Bool
	};

	#pragma pack(push, 1)
	class RemoteChannelDataStructureSettingsField {
		public:
			RemoteChannelDataStructureSettingsChannelType type = RemoteChannelDataStructureSettingsChannelType::Int;
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
				return "ds2";
			}

			void onRead(const NVSStream& stream) override {
				const auto dataTypeCount = stream.getUint8(_dataTypeCount, 0);

				fields.clear();

				if (dataTypeCount > 0) {
					const auto readFields = std::make_unique<RemoteChannelDataStructureSettingsField[]>(dataTypeCount);
					stream.getObject<RemoteChannelDataStructureSettingsField>(_dataTypeBlob, readFields.get(), dataTypeCount);

					for (int i = 0; i < dataTypeCount; ++i)
						fields.push_back(readFields[i]);
				}
			}

			void onWrite(const NVSStream& stream) override {
				stream.setUint8(_dataTypeCount, fields.size());

				if (fields.empty()) {
					stream.erase(_dataTypeBlob);
				}
				else {
					stream.setObject<RemoteChannelDataStructureSettingsField>(_dataTypeBlob, fields.data(), fields.size());
				}
			}

		private:
			constexpr static auto _dataTypeCount = "dtc";
			constexpr static auto _dataTypeBlob = "dtb";

	};

	class Settings {
		public:
			RemoteChannelDataStructureSettings remoteChannelDataStructure {};
			MotorsSettings motors {};

			void setup() {
				NVSSettings::setup();

				remoteChannelDataStructure.read();
				motors.read();
			}
	};
}