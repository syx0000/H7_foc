/**
 * @file    foc_kernel.h
 * @brief
 * @author  yxsui
 * @date    2025-08-01
 * @version 1.1
 */

#ifndef FOC_KERNEL_H_
#define FOC_KERNEL_H_

#include "foc_bsp.h"
#include "func_subprogram.h"
#include "func_trigonometric.h"

void clarke_transf(int32_t I_a, int32_t I_b, int32_t* alpha, int32_t* beta);
void park_transf(int32_t I_alpha, int32_t I_beta, int32_t* d, int32_t* q, int32_t theta);
void rev_park(int32_t d, int32_t q, int32_t Theta, int32_t* Valpha, int32_t* Vbeta);
void svpwm_calc(int32_t Valpha, int32_t Vbeta, uint32_t* CCR1, uint32_t* CCR2, uint32_t* CCR3);

Trig_Components get_sincos_value(int32_t Angle);
void limit_norm(int32_t* x, int32_t* y, int16_t limit);

#endif
