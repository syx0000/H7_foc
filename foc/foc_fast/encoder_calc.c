/**
 * @file    encoder_calc.c
 * @brief   FOC编码器计算（基于DPT双磁24位编码器）
 *          移植自 HPMicro PHU 项目 encoder.c 架构
 *          高速端（电机端）用 outer_raw，低速端（输出端）用 inner_raw
 */
#include "encoder_calc.h"
#include "foc_controller.h"
#include "encoder.h"       // Core/Inc/encoder.h - DPT驱动
#include <stdlib.h>
#include <stdio.h>

extern ControllerStruct controller_eyou;

/* 速度换算位移除数：24位编码器，2^14=16384（PHU 原为 19位时用 2^9=512） */
#define SPEED_SHIFT_DIV 16384.0f

/* 获取电机端角度原始值（inner，24位） */
static inline uint32_t get_motor_angle_raw(DPT_Angles *a) {
    DPT_GetLatestAngles_ISR(a);
    return a->outer_raw;
}

/* 获取输出端角度原始值（outer，24位） */
static inline uint32_t get_output_angle_raw(DPT_Angles *a) {
    return a->inner_raw;
}

/*******************************************************************************
  函数名: Encoder_data_Calculate
  描  述: 高速端编码器（电机端）相关计算：电角度/机械角度/速度
********************************************************************************/
void Encoder_data_Calculate(ControllerStruct* controller, uint16_t hz) {
    uint32_t temp;
    int32_t temp_error;
    int32_t dtheta_mech_temp;
    static uint8_t real_position_read = 0;
    static uint16_t error_count = 0;
    DPT_Angles dpt_angles;

    /* 一次性读出双编码器（减少 disable_irq 开销） */
    temp = get_motor_angle_raw(&dpt_angles);
    controller->now_mechposition = temp;

    /* 电角度：(NPP * temp) % 2^24，右移8位 → 0~65536 */
    controller->theta_elec_raw = (uint16_t)(((NPP * temp) % ENCODER_BIT) >> ENCODER_16BIT_DIV);
    if (controller->FlashData.InvertDirflag != 1) {
        controller->theta_elec = controller->theta_elec_raw - controller->FlashData.elec_offest_0;
    } else {
        controller->theta_elec = controller->theta_elec_raw - controller->FlashData.elec_offest_1;
    }
    if (controller->theta_elec < 0) {
        controller->theta_elec += 65536;
    } else if (controller->theta_elec > 65536) {
        controller->theta_elec -= 65536;
    }

    /* 计圈 */
    if ((int32_t)(temp - controller->old_angle_count) > ENCODER_BIT_HALF) {
        controller->circle_count -= 1;
    }
    if ((int32_t)(temp - controller->old_angle_count) < -ENCODER_BIT_HALF) {
        controller->circle_count += 1;
    }

    /* real_position 单位 1°/1024
       公式：(raw * 360) >> 14 + (circle * 360 << 10)
       注意用 uint64_t 防止 raw*360 在 24位时溢出 */
    controller->real_position_raw =
        (int32_t)(((uint64_t)temp * 360) >> ENCODER_10BIT_DIV) +
        (controller->circle_count * 360 << 10);
    controller->real_position = controller->real_position_raw - controller->FlashData.mech_offest;
    controller->old_angle_count = temp;

    /* 速度 */
    temp_error = controller->now_mechposition - controller->old_mechposition;
    if (temp_error < -ENCODER_BIT_HALF) {
        dtheta_mech_temp = (int32_t)(((temp_error + ENCODER_BIT) * 60) * (hz / SPEED_SHIFT_DIV));
    } else if (temp_error > ENCODER_BIT_HALF) {
        dtheta_mech_temp = (int32_t)(((temp_error - ENCODER_BIT) * 60) * (hz / SPEED_SHIFT_DIV));
    } else {
        dtheta_mech_temp = (int32_t)((temp_error * 60) * (hz / SPEED_SHIFT_DIV));
    }

    /* 均值滤波（malloc的buffer可能为NULL，加保护） */
    if (controller->Speed_Filter.FilterRun != NULL &&
        controller->Speed_Filter.buffer != NULL) {
        controller->Speed_Filter.FilterRun(&controller->Speed_Filter, dtheta_mech_temp);
        controller->dtheta_mech = controller->Speed_Filter.filtered;
    } else {
        controller->dtheta_mech = dtheta_mech_temp;
    }

    /* 旋转方向 */
    controller->ServoState.Bit.ServoRunDriction = (controller->dtheta_mech > 0) ? 1 : 0;
    controller->old_mechposition = controller->now_mechposition;

    /* 编码器异常检测（启动阶段暂时关闭，避免速度瞬态误报） */
    #if 0
    if (real_position_read > 6 && abs(controller->dtheta_mech) > (int32_t)DEFAULT_MAX_SPEED * 3) {
        if (++error_count > 2000) {
            controller->ServoErrFlag.Bit.EncoderErr = 1;
            error_count = 0;
        }
    } else {
        error_count = 0;
    }
    if (real_position_read < 10) real_position_read++;
    #else
    (void)real_position_read;
    (void)error_count;
    #endif
}

/*******************************************************************************
  函数名: Encoder_out_data_Calculate
  描  述: 低速端编码器（输出端）相关计算：位置 + 速度
********************************************************************************/
void Encoder_out_data_Calculate(ControllerStruct* controller, uint16_t hz) {
    static uint8_t out_first_run = 1;
    DPT_Angles dpt_angles;
    DPT_GetLatestAngles_ISR(&dpt_angles);
    uint32_t temp_raw = get_output_angle_raw(&dpt_angles);
    int32_t temp = (int32_t)temp_raw - controller->FlashData.mech_offest_out;

    /* 偏移后数据范围恢复 */
    if (temp > ENCODER_BIT_OUT) {
        temp -= ENCODER_BIT_OUT;
    } else if (temp < 0) {
        temp += ENCODER_BIT_OUT;
    }

    /* 首次运行：初始化 old_angle_count_out / real_position_out_pre，避免跳变保护卡死 */
    if (out_first_run) {
        controller->old_angle_count_out = temp;
        controller->real_position_out_pre =
            (int32_t)(((uint64_t)temp * 360) >> ENCODER_10BIT_DIV);
        controller->real_position_out = controller->real_position_out_pre;
        controller->old_angle_count_out_raw = temp_raw;
        out_first_run = 0;
        return;
    }

    /* 计圈 */
    if ((int32_t)(temp - controller->old_angle_count_out) > ENCODER_BIT_HALF_OUT) {
        controller->circle_count_out -= 1;
    } else if ((int32_t)(temp - controller->old_angle_count_out) < -ENCODER_BIT_HALF_OUT) {
        controller->circle_count_out += 1;
    }

    /* real_position_out 单位 1°/1024 */
    int32_t real_position_out_temp =
        (int32_t)(((uint64_t)temp * 360) >> ENCODER_10BIT_DIV) +
        (controller->circle_count_out * 360 << 10);

    /* 位置跳变保护 */
    if (abs(real_position_out_temp - controller->real_position_out_pre) > 1024) {
        controller->real_position_out = controller->real_position_out_pre;
    } else {
        controller->real_position_out = real_position_out_temp;
    }

    /* 速度：(real_position_out - pre) * hz * 减速比 101 / 6   (/6 = *60/360) */
    controller->dtheta_mech_out =
        (controller->real_position_out - controller->real_position_out_pre) * hz * 101 / 6;

    controller->real_position_out_pre = controller->real_position_out;
    controller->old_angle_count_out = temp;
    controller->old_angle_count_out_raw = temp_raw;
}

/*******************************************************************************
  函数名: Encoder_out_data_Reset
  描  述: 低速端编码器零位设置，开机根据机械限位定义初始圈数
********************************************************************************/
void Encoder_out_data_Reset(int32_t MaxPositionLimit, int32_t MinPositionLimit) {
    DPT_Angles dpt_angles;
    DPT_GetLatestAngles(&dpt_angles);
    uint32_t temp_raw = dpt_angles.inner_raw;
    int32_t temp = (int32_t)temp_raw - controller_eyou.FlashData.mech_offest_out;

    if (temp > ENCODER_BIT_OUT) {
        temp -= ENCODER_BIT_OUT;
    } else if (temp < 0) {
        temp += ENCODER_BIT_OUT;
    }

    int32_t real_position_out_temp = (int32_t)(((uint64_t)temp * 360) >> ENCODER_10BIT_DIV);

    /* 根据限位定义初始圈数 */
    if (real_position_out_temp > MaxPositionLimit + 2048) {
        controller_eyou.circle_count_out -= 1;
        printf("real_position_out circle_count_out -1\r\n");
    } else if (real_position_out_temp < MinPositionLimit - 2048) {
        controller_eyou.circle_count_out += 1;
        printf("real_position_out circle_count_out +1\r\n");
    }

    controller_eyou.real_position_out =
        (int32_t)(((uint64_t)temp * 360) >> ENCODER_10BIT_DIV) +
        (controller_eyou.circle_count_out * 360 << 10);
    controller_eyou.real_position_out_pre = controller_eyou.real_position_out;
    controller_eyou.old_angle_count_out = temp;
    controller_eyou.old_angle_count_out_raw = temp_raw;
    controller_eyou.position_ref = controller_eyou.real_position_out;
}
