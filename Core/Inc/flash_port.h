/**
 * @file    flash_port.h
 * @brief   STM32H743 内部Flash读写移植层
 *          使用 Bank2 Sector 7 (0x081E0000, 128KB) 存储参数
 */
#ifndef __FLASH_PORT_H__
#define __FLASH_PORT_H__

#include "main.h"
#include <stdint.h>

/* Bank2 Sector 7: 0x081E0000 ~ 0x081FFFFF (128KB) */
#define FLASH_USER_START_ADDR   0x081E0000U
#define FLASH_USER_SECTOR       FLASH_SECTOR_7
#define FLASH_USER_BANK         FLASH_BANK_2

/* Flash 写入粒度：H743 必须 32字节（256位）对齐 */
#define FLASH_WRITE_GRANULARITY 32U

/* 擦除用户扇区（整个128KB） */
HAL_StatusTypeDef Flash_EraseSector(void);

/* 写入数据到用户扇区（自动对齐到32字节边界）
   addr: 绝对地址（必须在 FLASH_USER_START_ADDR 范围内）
   data: 源数据指针
   len:  字节数（内部会补齐到32字节倍数） */
HAL_StatusTypeDef Flash_WriteData(uint32_t addr, const void *data, uint32_t len);

/* 从Flash读取数据（直接内存映射读取） */
void Flash_ReadData(uint32_t addr, void *buf, uint32_t len);

#endif /* __FLASH_PORT_H__ */
