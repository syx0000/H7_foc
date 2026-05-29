/**
 * @file    boot_crc32.c
 * @brief   IEEE 802.3 / zlib CRC32 (bit-by-bit, no table).
 *
 * Identical to Core/Src/flash_port.c::Flash_Crc32. Duplicated here so the
 * bootloader does not pull in any of the application's flash_port code.
 */
#include <stdint.h>

uint32_t boot_crc32(const void *data, uint32_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int i = 0; i < 8; i++) {
            crc = (crc >> 1) ^ (0xEDB88320u & -(int32_t)(crc & 1));
        }
    }
    return ~crc;
}
