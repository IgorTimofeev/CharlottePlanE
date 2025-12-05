#include "packetParser.h"

namespace pizda {
	uint8_t PacketParser::getCRC8(const uint8_t* buffer, size_t length) {
		uint8_t crc = 0xff;
		size_t i, j;

		for (i = 0; i < length; i++) {
			crc ^= buffer[i];

			for (j = 0; j < 8; j++) {
				if ((crc & 0x80) != 0)
					crc = static_cast<uint8_t>((crc << 1) ^ 0x31);
				else
					crc <<= 1;
			}
		}

		return crc;
	}

	bool PacketParser::checkCRC(const uint8_t* buffer, size_t length) {
		const auto checksum = getCRC8(buffer, length);
		const auto expectedChecksum = *(buffer + length);

		if (checksum != expectedChecksum) {
			ESP_LOGE("PacketParser", "checksum mismatch: got %d, expected %d", checksum, expectedChecksum);

			return false;
		}

		return true;
	}

	uint8_t PacketParser::readValueCountAndCheckCRC(ReadableBitStream& bitStream, uint8_t valueCountExpected, uint8_t valueCountBitCount, uint8_t valueBitCount) {
		auto valueCount = bitStream.readUint8(valueCountBitCount);

		ESP_LOGI("PacketParser", "value count: %d", valueCount);

		if (valueCount != valueCountExpected) {
			ESP_LOGE("PacketParser", "value count mismatch: got %d, expected %d", valueCount, valueCountExpected);
			return 0;
		}

		const auto dataBitCount = Packet::typeBitCount + valueCountBitCount + valueBitCount * valueCount;
		const uint8_t dataByteCount = (dataBitCount + 7) / 8;

		// CRC check
		if (!checkCRC(bitStream.getBuffer(), dataByteCount))
			return 0;

		return valueCount;
	}

	uint8_t PacketParser::parseOne(uint8_t* buffer) {
		auto packetPtr = buffer;

		ESP_LOGI("PacketParser", "----------------");

		// Header
		const auto headerValid = memcmp(packetPtr, Packet::header, Packet::headerByteCount) == 0;
		packetPtr += Packet::headerByteCount;

		if (!headerValid) {
			ESP_LOGE("PacketParser", "mismatched header: %s", packetPtr);

			return 0;
		}

		ReadableBitStream bitStream { packetPtr };

		// Type
		const auto packetType = static_cast<PacketType>(bitStream.readUint16(4));
		ESP_LOGI("PacketParser", "type: %d", static_cast<uint8_t>(packetType));

		// Payload parsing
		if (!onParse(bitStream, packetType)) {
			ESP_LOGI("PacketParser", "Parsing failed");
			
			return 0;
		}

		// Total length = header + payload + CRC
		const auto payloadLength = bitStream.getBytesRead();
		packetPtr += payloadLength + 1;

		const auto packetLength = packetPtr - buffer;

		ESP_LOGI("PacketParser", "payload length: %d, total length: %d", payloadLength, packetLength);
		ESP_LOGI("PacketParser", "----------------");

		return packetLength;
	}

	void PacketParser::parse(uint8_t* buffer, uint8_t length) {
		auto bufferEnd = buffer + length;

		while (true) {
			auto packetLength = parseOne(buffer);

			if (packetLength == 0)
				break;

			buffer += packetLength;

			if (buffer >= bufferEnd)
				break;
		}
	}
}