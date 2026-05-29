/**
 * @file    flash_port.c
 * @brief   STM32H743 内部Flash读写实现（Bank2 Sector 7）
 */
#include "flash_port.h"
#include <string.h>

HAL_StatusTypeDef Flash_EraseSector(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t err = 0;

    HAL_FLASH_Unlock();

    erase.TypeErase    = FLASH_TYPEERASE_SECTORS;
    erase.Banks        = FLASH_USER_BANK;
    erase.Sector       = FLASH_USER_SECTOR;
    erase.NbSectors    = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 2.7V~3.6V

    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &err);

    HAL_FLASH_Lock();
    return st;
}

HAL_StatusTypeDef Flash_WriteData(uint32_t addr, const void *data, uint32_t len)
{
    /* 地址必须 32 字节对齐 */
    if (addr & (FLASH_WRITE_GRANULARITY - 1)) return HAL_ERROR;
    /* 必须落在片内 Flash 范围 (Bank1 + Bank2, 2MB) 内.
       具体扇区是否擦过由调用者负责; 这里只挡明显越界的指针. */
    if (addr < 0x08000000U || addr >= 0x08200000U) return HAL_ERROR;

    HAL_FLASH_Unlock();

    HAL_StatusTypeDef st = HAL_OK;
    uint32_t remaining = len;
    const uint8_t *src = (const uint8_t *)data;
    uint32_t cur_addr = addr;

    /* 每次写 32 字节，不足补 0xFF */
    uint8_t buf[FLASH_WRITE_GRANULARITY];
    while (remaining > 0) {
        uint32_t chunk = (remaining >= FLASH_WRITE_GRANULARITY) ?
                          FLASH_WRITE_GRANULARITY : remaining;
        memset(buf, 0xFF, FLASH_WRITE_GRANULARITY);
        memcpy(buf, src, chunk);

        /* H7 写入：FLASH_TYPEPROGRAM_FLASHWORD 写 256 位 */
        st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, cur_addr, (uint32_t)(uintptr_t)buf);
        if (st != HAL_OK) break;

        cur_addr += FLASH_WRITE_GRANULARITY;
        src += chunk;
        remaining -= chunk;
    }

    HAL_FLASH_Lock();
    return st;
}

void Flash_ReadData(uint32_t addr, void *buf, uint32_t len)
{
    memcpy(buf, (const void *)(uintptr_t)addr, len);
}

uint32_t Flash_Crc32(const void *data, uint32_t len)
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
