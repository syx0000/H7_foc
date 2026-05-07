/**
 * @file    func_trigonometric.c
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#include "func_trigonometric.h"

ATTR_PLACE_AT_FAST_RAM_INIT const int16_t hSin_Cos_Table[256] = SIN_COS_TABLE;

/*******************************************************************************
  : sin_approx
    :
    :
  :
    :
********************************************************************************/
float sin_approx(float x) {
  int32_t xint = x;

  if (xint < -32 || xint > 32)
    return 0.0f;            // Stop here on error input (5 * 360 Deg)

  while (x > M_PIf)
    x -= (2.0f * M_PIf);    // always wrap input angle to -PI..PI

  while (x < -M_PIf)
    x += (2.0f * M_PIf);

  if (x > (0.5f * M_PIf))
    x = (0.5f * M_PIf) - (x - (0.5f * M_PIf));    // We just pick -90..+90 Degree
  else if (x < -(0.5f * M_PIf))
    x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);

  float x2 = x * x;
  return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

/*******************************************************************************
  : cos_approx
    :
    :
  :
    :
********************************************************************************/
float cos_approx(float x) {
  return sin_approx(x + (0.5f * M_PIf));
}

/*******************************************************************************
  : atan2_approx
    :
    :
  :
    :
********************************************************************************/
float atan2_approx(float y, float x) {
  float res, absX, absY;
  absX = fabsf(x);
  absY = fabsf(y);
  res  = MAX(absX, absY);

  if (res)
    res = MIN(absX, absY) / res;
  else
    res = 0.0f;

  res = -((((atanPolyCoef5 * res - atanPolyCoef4) * res - atanPolyCoef3) * res - atanPolyCoef2) * res - atanPolyCoef1) /
      ((atanPolyCoef7 * res + atanPolyCoef6) * res + 1.0f);

  if (absY > absX)
    res = (M_PIf / 2.0f) - res;

  if (x < 0)
    res = M_PIf - res;

  if (y < 0)
    res = -res;

  return res;
}

/*******************************************************************************
  : acos_approx
    :
    :
  :
    :
********************************************************************************/
float acos_approx(float x) {
  float xa     = fabsf(x);
  float result = sqrtf(1.0f - xa) * (1.5707288f + xa * (-0.2121144f + xa * (0.0742610f + (-0.0187293f * xa))));

  if (x < 0.0f)
    return M_PIf - result;
  else
    return result;
}

/*******************************************************************************
  : constrainf
    :
    :
  :
    :
********************************************************************************/
float constrainf(float amt, float low, float high) {
  if (amt < low)
    return low;
  else if (amt > high)
    return high;
  else
    return amt;
}

/*******************************************************************************
  : invSqrt
    :
    :
  :
    :
********************************************************************************/
float invSqrt(float x) {
  return 1.0f / sqrtf(x);
}

/*******************************************************************************
  : FloatToU32
    :
    :
  :
    : u32
********************************************************************************/
uint32_t FloatToU32(float dat) {
  uint8_t buf[4];
  buf[0] = ((uint8_t*)&dat)[0];
  buf[1] = ((uint8_t*)&dat)[1];
  buf[2] = ((uint8_t*)&dat)[2];
  buf[3] = ((uint8_t*)&dat)[3];
  return (buf[0] << 24) + (buf[1] << 16) + (buf[2] << 8) + buf[3];
}

/*******************************************************************************
  : U32ToFloat
    :
    :
  :
    : u32
********************************************************************************/
float U32ToFloat(uint32_t dat) {
  uint8_t buf[4];
  buf[0] = dat >> 24;
  buf[1] = dat >> 16;
  buf[2] = dat >> 8;
  buf[3] = dat & 0xff;
  return *((float*)buf);
}
