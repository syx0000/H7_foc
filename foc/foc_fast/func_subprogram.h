/**
 * @file    func_subprogram.h
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#ifndef FUNC_SUBPROGRAM_H
#define FUNC_SUBPROGRAM_H

#include "foc_bsp.h"

uint16_t qsqrt(uint32_t dwNumber);
void ShortDivOutputBin(uint32_t const input);
uint16_t endian_toggle(uint16_t val);
uint32_t calculate_crc(const uint8_t* data, size_t length);
#endif
