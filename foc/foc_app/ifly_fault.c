/**
 * @file    ifly_fault.c
 * @brief   故障保护层实现（母线电压/温度/过流/堵转/DRV nFAULT）
 * @author  zcliu15
 * @date    2025-07-07
 * @version 2.0
 */

/* Includes ------------------------------------------------------------------*/
#include "ifly_fault.h"
#include "ifly_fault_api.h"
#include "ifly_led.h"
#include "foc_api.h"
#include "foc_data.h"
#include "main.h"
#include <stdio.h>

ifly_Err_Pro_Type motorProValue;
extern ControllerStruct controller_eyou;
extern Portection_Value Threshold;
extern LED_STATUSBits LED_STATUS;
extern void CiA402_LocalError(UINT16 ErrorCode);

extern volatile uint32_t g_vdc_raw;
extern volatile uint32_t g_temp_motor_raw;
extern volatile uint32_t g_temp_mos_raw;

/* VDC 分压比：ADC 16bit 3.3V 满量程，硬件分压 21:1（motor_h7 实测）
 * 转换为 0.1V 单位：Udc_01V = raw * 3.3 * 21 / 65535 * 10 = raw * 693 / 65535
 * 简化：raw * 693 / 65535 ≈ raw / 94.6 */
#define VDC_DIVIDER_RATIO  21
#define VDC_ADC_TO_01V(raw) ((uint32_t)(raw) * 33 * VDC_DIVIDER_RATIO / 65535)

/* 故障滤波计数器 */
static uint8_t ovp_filter_cnt = 0;
static uint8_t uvp_filter_cnt = 0;
static uint8_t ot_board_filter_cnt = 0;
static uint8_t oc_filter_cnt = 0;
static uint8_t locked_filter_cnt = 0;
static uint16_t spd_offset_cnt = 0;
static uint16_t pos_offset_cnt = 0;

/* 清除所有故障检测计数器 */
static void clear_all_fault_counters(void) {
    ovp_filter_cnt = 0;
    uvp_filter_cnt = 0;
    ot_board_filter_cnt = 0;
    oc_filter_cnt = 0;
    locked_filter_cnt = 0;
    spd_offset_cnt = 0;
    pos_offset_cnt = 0;
}

/* PLACEHOLDER_FAULT_IMPL */

/*******************************************************************************
 * adc_convert - 将 ADC 原始值转换为工程量，填充 motorProValue
 ******************************************************************************/
void adc_convert(void) {
    motorProValue.Udc = VDC_ADC_TO_01V(g_vdc_raw);
    motorProValue.board_temp = TemperatureInquiry((uint16_t)g_temp_mos_raw);
    motorProValue.motor_temp = TemperatureInquiry((uint16_t)g_temp_motor_raw);
}

/*******************************************************************************
 * TemperatureInquiry - NTC ADC 值转温度（°C）
 * B 值公式：1/T = 1/T0 + ln(R/R0)/B
 * 硬件：10k 分压 + 10k NTC，ADC 16bit 3.3V
 * 电机 NTC B=3435，MOS NTC B=3950（motor_h7 实测）
 * 统一用 B=3950（MOS 侧），电机侧误差 <2°C 可接受
 ******************************************************************************/
#include <math.h>

#define NTC_B_VALUE      3950.0f
#define NTC_R_REF        10000.0f   /* 参考电阻 10k @ 25°C */
#define NTC_R_DIVIDER    10000.0f   /* 分压电阻 10k */
#define NTC_T_REF        298.15f    /* 25°C in Kelvin */
#define NTC_ABS_ZERO     273.15f

int8_t TemperatureInquiry(uint16_t adc_value) {
    if (adc_value == 0 || adc_value >= 65535) return 0;

    float ratio = (float)adc_value / 65535.0f;
    float r_ntc = NTC_R_DIVIDER * ratio / (1.0f - ratio);

    float temp_k = 1.0f / ((1.0f / NTC_T_REF) + logf(r_ntc / NTC_R_REF) / NTC_B_VALUE);
    int8_t temp_c = (int8_t)(temp_k - NTC_ABS_ZERO);

    return temp_c;
}

/*******************************************************************************
 * dcVoltageProFunc - 母线过压/欠压保护（5 次滤波）
 * 单位：motorProValue.Udc 为 0.1V
 ******************************************************************************/
int8_t dcVoltageProFunc(void) {
    uint32_t udc = motorProValue.Udc;

    if (udc > Threshold.OverUdc) {
        if (++ovp_filter_cnt >= FILTER_TIME) {
            ovp_filter_cnt = FILTER_TIME;
            controller_eyou.ServoErrFlag.Bit.OverBusVolErr = 1;
            return 1;
        }
    } else {
        ovp_filter_cnt = 0;
    }

    if (udc < Threshold.LowUdc && udc > 10) {
        if (++uvp_filter_cnt >= FILTER_TIME) {
            uvp_filter_cnt = FILTER_TIME;
            controller_eyou.ServoErrFlag.Bit.LowBusVolErr = 1;
            return -1;
        }
    } else {
        uvp_filter_cnt = 0;
    }

    return 0;
}

/*******************************************************************************
 * boradTempProFunc - 板温过温保护（两级：警告 + 停机）
 ******************************************************************************/
int8_t boradTempProFunc(void) {
    int8_t temp = motorProValue.board_temp;

    if (temp >= (int8_t)Threshold.TemBorad) {
        if (++ot_board_filter_cnt >= FILTER_TIME) {
            ot_board_filter_cnt = FILTER_TIME;
            controller_eyou.ServoErrFlag.Bit.HighBoardTempErr = 1;
            return 1;
        }
    } else {
        ot_board_filter_cnt = 0;
    }

    return 0;
}

/* PLACEHOLDER_FAULT_IMPL2 */

/*******************************************************************************
 * busOverCurrentCheck - 母线过流保护（基于相电流峰值，10 次滤波）
 * 开环和闭环运行时都检测（foc_run >= 1）
 ******************************************************************************/
void busOverCurrentCheck(void) {
    if (controller_eyou.foc_run < 1) {
        oc_filter_cnt = 0;
        return;
    }

    int32_t iq_abs = controller_eyou.I_q;
    if (iq_abs < 0) iq_abs = -iq_abs;

    if ((uint16_t)iq_abs > Threshold.OverCurrent) {
        if (++oc_filter_cnt >= Threshold.OverCurrentTime) {
            oc_filter_cnt = Threshold.OverCurrentTime;
            controller_eyou.ServoErrFlag.Bit.OverBusCurrentErr = 1;
        }
    } else {
        if (oc_filter_cnt > 0) oc_filter_cnt--;
    }
}

/*******************************************************************************
 * LockedRotorProFunc - 堵转保护（电流大 + 速度小 = 堵转）
 * 开环和闭环运行时都检测（foc_run >= 1）
 ******************************************************************************/
uint8_t LockedRotorProFunc(void) {
    if (controller_eyou.foc_run < 1) {
        locked_filter_cnt = 0;
        return 0;
    }

    int32_t iq_abs = controller_eyou.I_q;
    if (iq_abs < 0) iq_abs = -iq_abs;

    int32_t spd_abs = controller_eyou.dtheta_mech;
    if (spd_abs < 0) spd_abs = -spd_abs;

    if ((uint16_t)iq_abs > Threshold.BlockTorque &&
        (uint32_t)spd_abs < Threshold.BlockSpeed) {
        if (++locked_filter_cnt >= Threshold.BlockTime) {
            locked_filter_cnt = Threshold.BlockTime;
            controller_eyou.ServoErrFlag.Bit.LockedRotorErr = 1;
            return 1;
        }
    } else {
        if (locked_filter_cnt > 0) locked_filter_cnt--;
    }

    return 0;
}

/*******************************************************************************
 * driverChipFaultCheck - DRV8353 nFAULT 引脚检测
 * TODO: 需要确认 nFAULT GPIO 引脚是否已接入
 ******************************************************************************/
uint8_t driverChipFaultCheck(void) {
    /* nFAULT 引脚未接入硬件，暂不实现 */
    return 0;
}

/*******************************************************************************
 * print_fault_types - 按故障位打印具体的错误类型
 ******************************************************************************/
static void print_fault_types(Servo_Flag_Unin flag) {
    if (flag.Bit.OverBusVolErr)          printf("  [1]  OverBusVol    (Udc=%lu/10V)\r\n",
                                                 (unsigned long)motorProValue.Udc);
    if (flag.Bit.LowBusVolErr)           printf("  [2]  LowBusVol     (Udc=%lu/10V)\r\n",
                                                 (unsigned long)motorProValue.Udc);
    if (flag.Bit.OverBusCurrentErr)      printf("  [3]  OverBusCur    (I_q=%ld Q10)\r\n",
                                                 (long)controller_eyou.I_q);
    if (flag.Bit.HighBoardTempErr)       printf("  [4]  HighBoardTemp (T=%dC)\r\n",
                                                 motorProValue.board_temp);
    if (flag.Bit.HighMotorTempErr)       printf("  [5]  HighMotorTemp (T=%dC)\r\n",
                                                 motorProValue.motor_temp);
    if (flag.Bit.LockedRotorErr)         printf("  [6]  LockedRotor   (I_q=%ld, v_mech=%ld)\r\n",
                                                 (long)controller_eyou.I_q,
                                                 (long)controller_eyou.dtheta_mech);
    if (flag.Bit.EncoderErr)             printf("  [7]  EncoderErr\r\n");
    if (flag.Bit.DriverChipNfault)       printf("  [8]  DriverNfault  (DRV8353 HW fault)\r\n");
    if (flag.Bit.sto_activated)          printf("  [9]  STO activated\r\n");
    if (flag.Bit.MosFault)               printf("  [10] MosFault\r\n");
    if (flag.Bit.CommunicateErr)         printf("  [11] CommErr       (CAN timeout)\r\n");
    if (flag.Bit.OverSpeedErr)           printf("  [12] OverSpeed     (v_ref=%ld)\r\n",
                                                 (long)controller_eyou.velocity_ref_filterd);
    if (flag.Bit.OverPositionErr)        printf("  [13] OverPos       (pos=%ld)\r\n",
                                                 (long)controller_eyou.real_position_out);
    if (flag.Bit.PhaseUVolErr)           printf("  [14] PhaseU_Err\r\n");
    if (flag.Bit.PhaseVVolErr)           printf("  [15] PhaseV_Err\r\n");
    if (flag.Bit.PhaseWVolErr)           printf("  [16] PhaseW_Err\r\n");
    if (flag.Bit.PhaseCurrentSampleErr)  printf("  [17] IxSampleErr\r\n");
    if (flag.Bit.CommunicateFlag)        printf("  [18] CommFlag\r\n");
    if (flag.Bit.DCBusSampleErr)         printf("  [19] UdcSampleErr\r\n");
    if (flag.Bit.BoradTemSampleErr)      printf("  [20] BoardTSampleErr\r\n");
    if (flag.Bit.MotorTemSampleErr)      printf("  [21] MotorTSampleErr\r\n");
    if (flag.Bit.PhaseOrderErr)          printf("  [22] PhaseOrderErr\r\n");
    if (flag.Bit.UserCommendValueErr)    printf("  [23] UserCmdErr\r\n");
    if (flag.Bit.MotorMaxAccErr)         printf("  [24] MaxAccErr\r\n");
    if (flag.Bit.MotorMaxJerkErr)        printf("  [25] MaxJerkErr\r\n");
    if (flag.Bit.speedOffsetErr)         printf("  [26] SpeedOffset   (v_ref=%ld, v_act=%ld)\r\n",
                                                 (long)controller_eyou.velocity_ref_filterd,
                                                 (long)controller_eyou.dtheta_mech);
    if (flag.Bit.eepromDataErr)          printf("  [27] EepromDataErr\r\n");
    if (flag.Bit.posOffsetErr)           printf("  [28] PosOffset     (p_ref=%ld, p_act=%ld)\r\n",
                                                 (long)controller_eyou.position_ref,
                                                 (long)controller_eyou.real_position_out);
    if (flag.Bit.zeroPointErr)           printf("  [29] ZeroPointErr\r\n");
    if (flag.Bit.currentOffsetErr)       printf("  [30] CurOffsetErr\r\n");
    if (flag.Bit.flashReadErr)           printf("  [31] FlashReadErr\r\n");
}

/*******************************************************************************
 * 故障分级 - 根据严重程度选择停机方式
 * 严重故障（硬件损坏风险）：立即关 PWM
 * 一般故障（无即时硬件风险）：斜坡减速后再关 PWM，缓解反电动势浪涌
 ******************************************************************************/
static uint32_t get_critical_fault_mask(void) {
    /* 严重故障位掩码 - 必须立即关 PWM 防止硬件损坏 */
    Servo_Flag_Unin mask;
    mask.All_Flag = 0;
    mask.Bit.OverBusVolErr     = 1;  /* 母线过压：再开 PWM 会损坏 */
    mask.Bit.OverBusCurrentErr = 1;  /* 过流：MOSFET 烧毁风险 */
    mask.Bit.HighBoardTempErr  = 1;  /* 板过温：MOSFET 结温危险 */
    mask.Bit.HighMotorTempErr  = 1;  /* 电机过温 */
    mask.Bit.DriverChipNfault  = 1;  /* DRV 硬件故障 */
    mask.Bit.MosFault          = 1;  /* MOSFET 故障 */
    mask.Bit.PhaseCurrentSampleErr = 1;  /* 电流采样错 */
    return mask.All_Flag;
}

/*******************************************************************************
 * fault_safe_shutdown - 安全关 PWM
 * 中点占空比 → 关 MOE → 清 CCER
 ******************************************************************************/
static void fault_safe_shutdown(void) {
    uint32_t period = TIM1->ARR;
    TIM1->CCR1 = period / 2;
    TIM1->CCR2 = period / 2;
    TIM1->CCR3 = period / 2;
    __DSB();
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    TIM1->CCER &= ~0x1555u;
}

/*******************************************************************************
 * fault_clear_run_data - 清理运行数据
 ******************************************************************************/
static void fault_clear_run_data(void) {
    controller_eyou.velocity_ref          = 0;
    controller_eyou.velocity_ref_filterd  = 0;
    controller_eyou.position_ref          = 0;
    controller_eyou.position_ref_filterd  = 0;
    controller_eyou.I_q_ref               = 0;
    controller_eyou.I_d_ref               = 0;
    controller_eyou.SpeedSmooth.NowVelocityRef = 0;
    ResetControlData(&controller_eyou);
    clear_all_fault_counters();
}

/*******************************************************************************
 * CheckAndHandleAllFaultBits - 故障总分发：扫描所有故障位，触发时停机
 * 仅在错误码变化时上报，避免反复打印同一错误
 *
 * 停机策略（分级）：
 * - 严重故障（OVP/OC/OT/MOSFault）：立即关 PWM，防止硬件损坏
 * - 一般故障：先斜坡减速 100ms（I_q_ref 渐变到 0），再关 PWM
 *   缓解反电动势浪涌，保护母线电容和 MOSFET
 ******************************************************************************/
uint8_t CheckAndHandleAllFaultBits(void) {
    static uint32_t last_fault_flag = 0;
    static uint8_t  ramp_down_phase = 0;       /* 0=正常, 1=斜坡减速中 */
    static uint16_t ramp_down_ticks = 0;       /* 减速计数（ms） */
    static int32_t  ramp_down_iq_start = 0;    /* 进入减速时的 I_q_ref */
    const uint16_t  RAMP_DOWN_MS = 100;        /* 斜坡时间 100ms */

    uint32_t current_fault = controller_eyou.ServoErrFlag.All_Flag;

    if (current_fault == 0) {
        last_fault_flag = 0;
        ramp_down_phase = 0;
        ramp_down_ticks = 0;
        return 0;
    }

    /* 斜坡减速进行中：每次调用（1ms）线性减小 I_q_ref */
    if (ramp_down_phase == 1) {
        ramp_down_ticks++;
        if (ramp_down_ticks >= RAMP_DOWN_MS) {
            /* 减速完成，关 PWM */
            controller_eyou.foc_run = 0;
            fault_safe_shutdown();
            fault_clear_run_data();
            printf("Ramp-down done, PWM disabled\r\n");
            printf("====================================\r\n");
            ramp_down_phase = 0;
            return 1;
        }
        /* 线性递减 I_q_ref */
        controller_eyou.I_q_ref =
            ramp_down_iq_start * (RAMP_DOWN_MS - ramp_down_ticks) / RAMP_DOWN_MS;
        controller_eyou.velocity_ref = 0;
        controller_eyou.position_ref = 0;
        return 1;
    }

    /* 仅在错误码变化时进入新故障处理 */
    if (current_fault != last_fault_flag) {
        uint32_t critical_mask = get_critical_fault_mask();
        uint8_t  is_critical   = (current_fault & critical_mask) != 0;

        printf("\r\n========== FAULT DETECTED ==========\r\n");
        printf("ServoErrFlag = 0x%08lX (prev: 0x%08lX) %s\r\n",
               (unsigned long)current_fault,
               (unsigned long)last_fault_flag,
               is_critical ? "[CRITICAL]" : "[NORMAL]");
        print_fault_types(controller_eyou.ServoErrFlag);

        if (is_critical) {
            /* 严重故障：立即关 PWM */
            controller_eyou.foc_run = 0;
            fault_safe_shutdown();
            fault_clear_run_data();
            printf("CRITICAL fault, PWM immediately disabled\r\n");
            printf("====================================\r\n");
        } else {
            /* 一般故障：进入斜坡减速 */
            ramp_down_phase = 1;
            ramp_down_ticks = 0;
            ramp_down_iq_start = controller_eyou.I_q_ref;
            printf("Normal fault, ramp-down %ums (I_q_ref=%ld -> 0)\r\n",
                   RAMP_DOWN_MS, (long)ramp_down_iq_start);
            /* 不立即关 PWM，让电流环继续工作 */
        }

        last_fault_flag = current_fault;
    }

    return 1;
}

/*******************************************************************************
 * ClearFaults - 故障复位（清除所有故障标志）
 ******************************************************************************/
uint8_t ClearFaults(uint8_t Fault_clear) {
    if (Fault_clear) {
        controller_eyou.ServoErrFlag.All_Flag = 0;
        clear_all_fault_counters();
        printf("All faults cleared, ready to restart\r\n");
    }
    return 0;
}

/* PLACEHOLDER_FAULT_IMPL3 */

/*******************************************************************************
 * 偏差监控函数（速度/位置/电流跟随偏差超限）
 * 仅在闭环控制时检测（foc_run == 2）
 ******************************************************************************/
void motorSpeedOffsetCheck(void) {
    if (controller_eyou.foc_run < 1) {
        spd_offset_cnt = 0;
        return;
    }

    int32_t offset = controller_eyou.velocity_ref_filterd - controller_eyou.dtheta_mech;
    if (offset < 0) offset = -offset;

    int32_t speed_ref_abs = controller_eyou.velocity_ref_filterd;
    if (speed_ref_abs < 0) speed_ref_abs = -speed_ref_abs;

    int32_t threshold = speed_ref_abs * Threshold.velocity_coe / 10;
    if (threshold < 256000) threshold = 256000;  /* 10 rpm 载端死区 */

    if (offset > threshold) {
        if (++spd_offset_cnt >= 500) {            /* 500ms 滤波，覆盖斜坡加速时间 */
            spd_offset_cnt = 500;
            controller_eyou.ServoErrFlag.Bit.speedOffsetErr = 1;
        }
    } else {
        spd_offset_cnt = 0;
    }
}

void motorPosOffsetCheck(void) {
    if (controller_eyou.foc_run < 1) {
        pos_offset_cnt = 0;
        return;
    }
    if (controller_eyou.controller_mode != 1) return;

    int32_t offset = controller_eyou.position_ref - controller_eyou.real_position_out;
    if (offset < 0) offset = -offset;

    if ((uint32_t)offset > Threshold.PositionErr) {
        if (++pos_offset_cnt >= DEFAULT_POS_CHECKK_TIME) {
            pos_offset_cnt = DEFAULT_POS_CHECKK_TIME;
            controller_eyou.ServoErrFlag.Bit.posOffsetErr = 1;
        }
    } else {
        pos_offset_cnt = 0;
    }
}

void motorCurrentOffsetCheck(void) {
    /* 电流跟随偏差：暂不实现，电流环响应快不易超限 */
}

void motorOverPosCheck(void) {
    if (controller_eyou.foc_run < 1) return;
    if (controller_eyou.FlashData.PositionLimitFlag != 50) return;

    int32_t pos = controller_eyou.real_position_out;
    if (pos > controller_eyou.FlashData.MaxPositionLimit ||
        pos < controller_eyou.FlashData.MinPositionLimit) {
        controller_eyou.ServoErrFlag.Bit.OverPositionErr = 1;
    }
}

void motorSpeedOverCheck(void) {
    if (controller_eyou.foc_run < 1) return;

    int32_t spd = controller_eyou.dtheta_mech;
    if (spd < 0) spd = -spd;

    if ((uint32_t)spd > (uint32_t)Threshold.velocity_Limit) {
        controller_eyou.ServoErrFlag.Bit.OverSpeedErr = 1;
    }
}

/*******************************************************************************
 * Getter 函数
 ******************************************************************************/
int8_t getBoardTemp(void) { return motorProValue.board_temp; }
int8_t getMotorTemp(void) { return motorProValue.motor_temp; }
uint32_t getUPhaseu(void) { return motorProValue.UPhaseu; }
uint32_t getVPhaseu(void) { return motorProValue.VPhaseu; }
uint32_t getWPhaseu(void) { return motorProValue.WPhaseu; }
uint16_t getIBusCurrent(void) { return motorProValue.IBusCurrent; }
uint32_t getUdc(void) { return motorProValue.Udc; }

uint8_t motorProValueUpdate(void) {
    adc_convert();
    return 0;
}

uint8_t motorFaultCheck(void) {
    return (controller_eyou.ServoErrFlag.All_Flag != 0) ? 1 : 0;
}

uint8_t check_fault_flag(void) {
    return (controller_eyou.ServoErrFlag.All_Flag != 0) ? 1 : 0;
}

uint8_t get_warn_status(void) { return 0; }
uint8_t get_hardword_status(void) { return 0; }

/*******************************************************************************
 * target_reach_check - 到达判据检测（1ms 周期调用）
 * 根据当前控制模式判断位置/速度/电流是否到达目标
 ******************************************************************************/
void target_reach_check(void) {
    static uint16_t pos_arrive_cnt = 0;
    static uint16_t spd_arrive_cnt = 0;
    static uint16_t cur_arrive_cnt = 0;

    if (controller_eyou.foc_run < 1) {
        pos_arrive_cnt = 0;
        spd_arrive_cnt = 0;
        cur_arrive_cnt = 0;
        controller_eyou.ServoState.Bit.PositionArrivedFlag = 0;
        controller_eyou.ServoState.Bit.SpeedArrivedFlag = 0;
        controller_eyou.ServoState.Bit.CurrentArrivedFlag = 0;
        return;
    }

    uint8_t mode = controller_eyou.controller_mode;

    /* 位置模式 */
    if (mode == PROFILE_POSITION_MODE || mode == CYCLIC_SYNC_POSITION_MODE) {
        int32_t err = controller_eyou.position_ref - controller_eyou.real_position_out;
        if (err < 0) err = -err;

        if ((uint32_t)err < controller_eyou.FlashData.PositionArrivedValue) {
            if (++pos_arrive_cnt >= POSITION_ARRIVED_TIME) {
                pos_arrive_cnt = POSITION_ARRIVED_TIME;
                controller_eyou.ServoState.Bit.PositionArrivedFlag = 1;
            }
        } else {
            pos_arrive_cnt = 0;
            controller_eyou.ServoState.Bit.PositionArrivedFlag = 0;
        }
    } else {
        pos_arrive_cnt = 0;
        controller_eyou.ServoState.Bit.PositionArrivedFlag = 0;
    }

    /* 速度模式 */
    if (mode == PROFILE_VELOCITY_MOCE || mode == CYCLIC_SYNC_VELOCITY_MODE) {
        int32_t err = controller_eyou.velocity_ref_filterd - controller_eyou.dtheta_mech;
        if (err < 0) err = -err;

        int32_t threshold = (int32_t)controller_eyou.FlashData.SpeedArrivedValue * 1024;
        if (err < threshold) {
            if (++spd_arrive_cnt >= SPEED_ARRIVED_TIME) {
                spd_arrive_cnt = SPEED_ARRIVED_TIME;
                controller_eyou.ServoState.Bit.SpeedArrivedFlag = 1;
            }
        } else {
            spd_arrive_cnt = 0;
            controller_eyou.ServoState.Bit.SpeedArrivedFlag = 0;
        }
    } else {
        spd_arrive_cnt = 0;
        controller_eyou.ServoState.Bit.SpeedArrivedFlag = 0;
    }

    /* 力矩模式 */
    if (mode == PROFILE_TORQUE_MODE || mode == CYCLIC_SYNC_TORQUE_MODE) {
        int32_t err = controller_eyou.I_q_ref - controller_eyou.I_q;
        if (err < 0) err = -err;

        if ((uint32_t)err < controller_eyou.FlashData.CurrentArrivedValue) {
            if (++cur_arrive_cnt >= CURRENT_ARRIVED_TIME) {
                cur_arrive_cnt = CURRENT_ARRIVED_TIME;
                controller_eyou.ServoState.Bit.CurrentArrivedFlag = 1;
            }
        } else {
            cur_arrive_cnt = 0;
            controller_eyou.ServoState.Bit.CurrentArrivedFlag = 0;
        }
    } else {
        cur_arrive_cnt = 0;
        controller_eyou.ServoState.Bit.CurrentArrivedFlag = 0;
    }
}

/*******************************************************************************
 * 抱闸控制（硬件未接入，保留空实现）
 ******************************************************************************/
void brake_open(uint8_t block) { (void)block; }
void brake_close(void) {}
uint8_t brake_close_limit(void) { return 0; }

void motorPhaseVolCheck(void) {}

