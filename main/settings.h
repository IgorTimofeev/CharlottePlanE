#pragma once

#include <YOBANVS/main.h>
#include "hardware/motor.h"

namespace pizda {
	using namespace YOBA;

	class MotorsSettings : public NVSSettings {
		public:
			MotorSettings leftEngine {};
			MotorSettings rightEngine {};
			
			MotorSettings leftAileron {};
			MotorSettings rightAileron {};

			MotorSettings leftTail {};
			MotorSettings rightTail {};

			MotorSettings leftFlap {};
			MotorSettings rightFlap {};

		protected:
			const char* getNamespace() override {
				return "scc";
			}

			void onRead(const NVSStream& stream) override {
				// Engines
				leftEngine.min = stream.getUint16(_leftEngineMin, 1000);
				leftEngine.max = stream.getUint16(_leftEngineMax, 2000);
				leftEngine.offset = stream.getInt16(_leftEngineOffset, 0);

				rightEngine.min = stream.getUint16(_rightEngineMin, 1000);
				rightEngine.max = stream.getUint16(_rightEngineMax, 2000);
				rightEngine.offset = stream.getInt16(_rightEngineOffset, 0);
				
				// Ailerons
				leftAileron.min = stream.getUint16(_leftAileronMin, 1000);
				leftAileron.max = stream.getUint16(_leftAileronMax, 2000);
				leftAileron.offset = stream.getInt16(_leftAileronOffset, 0);

				rightAileron.min = stream.getUint16(_rightAileronMin, 1000);
				rightAileron.max = stream.getUint16(_rightAileronMax, 2000);
				rightAileron.offset = stream.getInt16(_rightAileronOffset, 0);

				// Tail
				leftFlap.min = stream.getUint16(_leftFlapMin, 1000);
				leftFlap.max = stream.getUint16(_leftFlapMax, 2000);
				leftFlap.offset = stream.getInt16(_leftFlapOffset, 0);

				rightFlap.min = stream.getUint16(_rightFlapMin, 1000);
				rightFlap.max = stream.getUint16(_rightFlapMax, 2000);
				rightFlap.offset = stream.getInt16(_rightFlapOffset, 0);

				// Flaps
				leftTail.min = stream.getUint16(_leftFlapMin, 1000);
				leftTail.max = stream.getUint16(_leftFlapMax, 2000);
				leftTail.offset = stream.getInt16(_leftFlapOffset, 0);

				rightTail.min = stream.getUint16(_rightTailMin, 1000);
				rightTail.max = stream.getUint16(_rightTailMax, 2000);
				rightTail.offset = stream.getInt16(_rightTailOffset, 0);
			}

			void onWrite(const NVSStream& stream) override {
				// Engines
				stream.setUint16(_leftEngineMin, leftEngine.min);
				stream.setUint16(_leftEngineMax, leftEngine.max);
				stream.setInt16(_leftEngineOffset, leftEngine.offset);

				stream.setUint16(_rightEngineMin, rightEngine.min);
				stream.setUint16(_rightEngineMax, rightEngine.max);
				stream.setInt16(_rightEngineOffset, rightEngine.offset);
				
				// Ailerons
				stream.setUint16(_leftAileronMin, leftAileron.min);
				stream.setUint16(_leftAileronMax, leftAileron.max);
				stream.setInt16(_leftAileronOffset, leftAileron.offset);

				stream.setUint16(_rightAileronMin, rightAileron.min);
				stream.setUint16(_rightAileronMax, rightAileron.max);
				stream.setInt16(_rightAileronOffset, rightAileron.offset);

				// Tail
				stream.setUint16(_leftTailMin, leftTail.min);
				stream.setUint16(_leftTailMax, leftTail.max);
				stream.setInt16(_leftTailOffset, leftTail.offset);

				stream.setUint16(_rightTailMin, rightTail.min);
				stream.setUint16(_rightTailMax, rightTail.max);
				stream.setInt16(_rightTailOffset, rightTail.offset);

				// Flaps
				stream.setUint16(_leftFlapMin, leftFlap.min);
				stream.setUint16(_leftFlapMax, leftFlap.max);
				stream.setInt16(_leftFlapOffset, leftFlap.offset);

				stream.setUint16(_rightFlapMin, rightFlap.min);
				stream.setUint16(_rightFlapMax, rightFlap.max);
				stream.setInt16(_rightFlapOffset, rightFlap.offset);
			}

			private:
				constexpr static auto _leftEngineMin = "lem";
				constexpr static auto _leftEngineMax = "lex";
				constexpr static auto _leftEngineOffset = "leo";
	
				constexpr static auto _rightEngineMin = "rem";
				constexpr static auto _rightEngineMax = "rex";
				constexpr static auto _rightEngineOffset = "reo";
			
				constexpr static auto _leftAileronMin = "lam";
				constexpr static auto _leftAileronMax = "lax";
				constexpr static auto _leftAileronOffset = "lao";

				constexpr static auto _rightAileronMin = "ram";
				constexpr static auto _rightAileronMax = "rax";
				constexpr static auto _rightAileronOffset = "rao";

				constexpr static auto _leftTailMin = "ltm";
				constexpr static auto _leftTailMax = "ltx";
				constexpr static auto _leftTailOffset = "lto";

				constexpr static auto _rightTailMin = "rtm";
				constexpr static auto _rightTailMax = "rtx";
				constexpr static auto _rightTailOffset = "rto";

				constexpr static auto _leftFlapMin = "lfm";
				constexpr static auto _leftFlapMax = "lfx";
				constexpr static auto _leftFlapOffset = "lfo";

				constexpr static auto _rightFlapMin = "rfm";
				constexpr static auto _rightFlapMax = "rfx";
				constexpr static auto _rightFlapOffset = "rfo";
		};

	class Settings {
		public:
			MotorsSettings motors {};

			void setup() {
				NVSSettings::setup();

				motors.read();
			}
	};
}