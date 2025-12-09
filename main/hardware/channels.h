#pragma once

#include <vector>
#include <array>
#include <optional>
#include <esp_log.h>
#include "motor.h"

namespace pizda {
	enum class ChannelDataType : uint8_t {
		unsignedInteger,
		boolean
	};

	class Channel {
		public:
			explicit Channel(ChannelDataType dataType) : dataType(dataType) {

			}

			ChannelDataType getDataType() const {
				return dataType;
			}

		private:
			ChannelDataType dataType;
	};

	template<typename TValue>
	class ValueChannel : public Channel {
		public:
			explicit ValueChannel(ChannelDataType dataType) : Channel(dataType) {

			}

			TValue getValue() const {
				return _value;
			}

			void setValue(TValue value) {
				_value = value;
			}

		private:
			TValue _value {};
	};


	template<typename TValue>
	class NumericValueChannel : public ValueChannel<TValue> {
		public:
			NumericValueChannel(ChannelDataType dataType, uint8_t bitDepth) : ValueChannel<TValue>(dataType), _bitDepth(bitDepth) {

			}

			uint8_t getBitDepth() const {
				return _bitDepth;
			}

		private:
			uint8_t _bitDepth;
	};

	class UintChannel : public NumericValueChannel<uint32_t> {
		public:
			UintChannel(uint8_t bitDepth) : NumericValueChannel<uint32_t>(ChannelDataType::unsignedInteger, bitDepth)  {

			}
	};

	class BoolChannel : public ValueChannel<bool> {
		public:
			BoolChannel() : ValueChannel<bool>(ChannelDataType::boolean)  {

			}
	};

	class Channels {
		public:
			std::array<Channel*, 255> instances {};

			void setup();
			void updateFromSettings();
			void onValueUpdated();

		private:
			bool checkChannel(uint8_t channelIndex, ChannelDataType dataType);
			UintChannel* checkUintChannel(uint8_t channelIndex, uint8_t bitDepth);
			BoolChannel* checkBoolChannel(uint8_t channelIndex);
			void checkAndSetMotor(uint8_t channelIndex, ConfiguredMotor* motor);
	};
}