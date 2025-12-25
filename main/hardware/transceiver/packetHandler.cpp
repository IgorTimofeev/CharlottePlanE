#include "packetHandler.h"

#include <esp_log.h>
#include "hardware/transceiver/transceiver.h"

namespace pizda {
	uint8_t PacketHandler::getCRC8(const uint8_t* buffer, size_t length) {
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
	
	bool PacketHandler::validatePayloadChecksumAndLength(BitStream& stream, size_t dataBitCount, uint8_t payloadLength) {
		const uint8_t expectedPayloadLengthBytes = divideCeiling<uint8_t>(Packet::typeLengthBits + dataBitCount, 8);
		
		// Length
		if (payloadLength != expectedPayloadLengthBytes) {
			ESP_LOGE(_logTag, "payload length mismatch: got %d, expected %d", payloadLength, expectedPayloadLengthBytes);
			
			return false;
		}
		
		const auto checksum = getCRC8(stream.getBuffer(), expectedPayloadLengthBytes);
		const auto expectedChecksum = *(stream.getBuffer() + expectedPayloadLengthBytes);
		
		// Checksum
		if (checksum != expectedChecksum) {
			ESP_LOGE(_logTag, "checksum mismatch: got %d, expected %d", checksum, expectedChecksum);
			
			return false;
		}
		
		return true;
	}
	
	bool PacketHandler::read(uint8_t* buffer, uint8_t length) {
//		ESP_LOGI(_logTag, "-------- Begin --------");
		
		// Length check
		if (length < 1) {
			ESP_LOGE(_logTag, "failed to read packet: length %d is too small to fit any data", length);
			
			return false;
		}
		
		BitStream stream { buffer };
		
		// Type
		const auto packetType = static_cast<PacketType>(stream.readUint16(Packet::typeLengthBits));
		
		// Payload length = totalLength  - checksumLength
		const uint8_t payloadLength = length - Packet::checksumLengthBytes;
		
		if (!readPacket(stream, packetType, payloadLength))
			return false;
		
		return true;
	}
	
	bool PacketHandler::write(uint8_t* buffer, PacketType packetType, uint8_t& length) {
		BitStream stream { buffer };
		
		// Type
		stream.writeUint8(static_cast<uint8_t>(packetType), Packet::typeLengthBits);
		
		// Body
		if (!writePacket(stream, packetType))
			return false;
		
		const auto payloadLength = stream.getBytesProcessed();
		
		// Checksum
		const auto checksum = getCRC8(stream.getBuffer(), payloadLength);
		stream.nextByte();
		stream.writeUint8(checksum, Packet::checksumLengthBytes * 8);
		
		length = static_cast<uint8_t>(payloadLength + Packet::checksumLengthBytes);
		
		return true;
	}
}