#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void pnx_crc16_append(uint8_t* message, uint32_t length);
uint32_t pnx_crc16_verify(const uint8_t* message, uint32_t length);

#ifdef __cplusplus
}
#endif
