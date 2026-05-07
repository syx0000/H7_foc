/**
 * @file    func_subprogram.c
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#include "func_subprogram.h"

/*******************************************************************************
  : qsqrt()
    : uint32_t dwNumber
    :
  :
    : MSX(~1.6us) (+--1)   ** by tongwenzou20100812
********************************************************************************/
uint16_t qsqrt(uint32_t dwNumber) {
  int8_t i;
  uint32_t dwSquareRoot;

  if (dwNumber == 0)
    return (0);

  if (dwNumber <= 4194304)
    dwSquareRoot = (dwNumber >> 10) + 63;
  else if (dwNumber <= 134217728)
    dwSquareRoot = (dwNumber >> 12) + 255;
  else
    dwSquareRoot = (dwNumber >> 14) + 1023;

  for (i = 0; i < 5; i++)
    dwSquareRoot = (dwSquareRoot + dwNumber / dwSquareRoot) >> 1;

  return ((uint16_t)dwSquareRoot);
}

/*******************************************************************************
  : ShortDivOutputBin()
    : uint32_t const input
    :
  :
    : y
********************************************************************************/
void ShortDivOutputBin(uint32_t const input) {
  uint8_t temp[33] = {0};
  (void)temp;

  // printf("0b");
  for (int i = 31; i >= 0; i--) {
    temp[i] = (input >> i) & 0x01;
    //        printf("%d", temp[i]);
    //        if(i % 4 == 0 && i != 0)
    //            printf(" ");
  }

  // printf("\n");
}

/*******************************************************************************
  : endian_toggle
    :
    :
  :
    :
********************************************************************************/
uint16_t endian_toggle(uint16_t val) {
  uint16_t res;
  res = (val << 8) | (val >> 8);
  return res;
}
/*******************************************************************************
  函数名:calculate_crc
  输  入:
  输  出:
  子函数:
  描  述: 计算crc32值
********************************************************************************/
uint32_t calculate_crc(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;

  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
    }
  }
  return ~crc;
}
