#include "crc_c.h"

#include "crc.hpp"

extern "C" void pnx_crc16_append(uint8_t* message, uint32_t length) {
  crc::append_crc16_checksum(message, length);
}

extern "C" uint32_t pnx_crc16_verify(const uint8_t* message, uint32_t length) {
  return crc::verify_crc16_checksum(message, length);
}
