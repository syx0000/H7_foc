/**
 * @file    encoder_calc.h
 * @brief   FOC编码器计算（基于DPT双磁24位编码器）
 *          架构移植自 HPMicro PHU 项目 encoder.c
 *          单位沿用PHU定点格式：position=1°/1024，速度=1/1024rpm，theta_elec=0~65536
 */
#ifndef __ENCODER_CALC_H__
#define __ENCODER_CALC_H__

#include "foc_bsp.h"
#include "foc_controller.h"

/* 高速端（电机端）编码器计算：电角度 + 机械角度 + 速度 */
void Encoder_data_Calculate(ControllerStruct* controller, uint16_t hz);

/* 低速端（输出端）编码器计算：位置 + 速度 */
void Encoder_out_data_Calculate(ControllerStruct* controller, uint16_t hz);

/* 低速端编码器零位设置（开机根据限位定义初始圈数） */
void Encoder_out_data_Reset(int32_t MaxPositionLimit, int32_t MinPositionLimit);

#endif /* __ENCODER_CALC_H__ */
