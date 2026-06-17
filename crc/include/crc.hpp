#pragma once

#include <cstdint>

namespace crc {

uint16_t get_crc16_checksum(const uint8_t* message, uint32_t length, uint16_t seed);
uint32_t verify_crc16_checksum(const uint8_t* message, uint32_t length);
void append_crc16_checksum(uint8_t* message, uint32_t length);

uint8_t get_crc8_checksum(const uint8_t* message, uint16_t length, uint8_t seed);
uint32_t verify_crc8_checksum(const uint8_t* message, uint16_t length);
void append_crc8_checksum(uint8_t* message, uint16_t length);

}  // namespace crc
