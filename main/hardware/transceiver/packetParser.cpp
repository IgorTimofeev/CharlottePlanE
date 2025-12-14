#include "packetParser.h"
#include "logger.h"

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

	bool PacketParser::validateChecksum(const uint8_t* buffer, size_t dataBitCount) {
		const uint8_t dataByteCount = (Packet::typeBitCount + dataBitCount + 7) / 8;

		const auto checksum = getCRC8(buffer, dataByteCount);
		const auto expectedChecksum = *(buffer + dataByteCount);

		if (checksum != expectedChecksum) {
			Logger::error(_logTag, "checksum mismatch: got %d, expected %d", checksum, expectedChecksum);

			return false;
		}

		return true;
	}

	uint8_t PacketParser::readValueCountAndValidateChecksum(BitStream& bitStream, uint8_t valueCountBitCount, uint8_t valueBitCount) {
		auto valueCount = bitStream.readUint8(valueCountBitCount);

		Logger::info(_logTag, "value count: %d", valueCount);

		// CRC check
		if (!validateChecksum(bitStream.getBuffer(), valueCountBitCount + valueBitCount * valueCount))
			return 0;

		return valueCount;
	}

	uint8_t PacketParser::parseOne(uint8_t* buffer) {
		auto packetPtr = buffer;

		Logger::info(_logTag, "-------- Begin --------");

		// Header
		const auto headerValid = memcmp(packetPtr, Packet::header, Packet::headerByteCount) == 0;
		packetPtr += Packet::headerByteCount;

		if (!headerValid) {
			Logger::error(_logTag, "mismatched header: %s", packetPtr);

			return 0;
		}

		BitStream bitStream {packetPtr };

		// Type
		const auto packetType = static_cast<PacketType>(bitStream.readUint16(4));
		Logger::info(_logTag, "packet type: %d", static_cast<uint8_t>(packetType));

		// Payload parsing
		if (!onParse(bitStream, packetType)) {
			Logger::info(_logTag, "parsing failed");
			
			return 0;
		}

		// Total length = header + payload + CRC
		const auto payloadLength = bitStream.getBytesRead();
		packetPtr += payloadLength + 1;

		const auto packetLength = packetPtr - buffer;

		Logger::info(_logTag, "payload length: %d, total length: %d", payloadLength, packetLength);
		Logger::info(_logTag, "-------- End --------");

		return packetLength;
	}

	bool PacketParser::parse(uint8_t* buffer, uint8_t length) {
		auto bufferEnd = buffer + length;

		while (true) {
			auto packetLength = parseOne(buffer);

			if (packetLength == 0)
				return false;

			buffer += packetLength;

			if (buffer >= bufferEnd)
				break;
		}

		return true;
	}
}