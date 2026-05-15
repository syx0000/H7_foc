/**
 * @file    foc_bsp.c
 * @brief   模块功能描述
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#include "foc_bsp.h"
#include "foc_api.h"
#include "foc_data.h"
#include "foc_controller.h"
#include "func_errMes.h"
#include "ifly_fault.h"
#include "ifly_fault_api.h"
#include "ifly_led.h"
#include "ifly_test.h"
#include "tim.h"
#include "adc.h"
#include "encoder.h"
#include "flash_port.h"
#include "fdcan.h"
#include "can_wly.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* main.c 中定义的开环测试参数（logid 120 用） */
extern uint8_t open_loop_mode;
extern int16_t v_d_test;
extern int16_t v_q_test;

uint8_t dbgRecvBuf[1024];
volatile uint16_t usart_rx_len = 0;
volatile uint16_t dbgLogFlag   = 0;
volatile uint16_t logPriodMs   = 1;
volatile uint16_t testLogFlag  = 0;

extern ifly_Err_Pro_Type motorProValue;
extern ErrMessgeStruct ErrMessge[ERRMESSGECOUNT];

extern Portection_Value Threshold;

extern volatile uint16_t pp_diag_udc_peak;
extern volatile uint16_t pp_diag_ticks_left;

uint8_t NPP = 0;

void seiInterruptReset(void) {
}

void led_init(void) {
}

void break_motor_operation_init(void) {
}

void sto_motor_operation_init(void) {
}

void isr_gpio(void) {
}

void pwm_pins_init(void) {
}

void isr_pwm0_counter(void) {
}

void pwmv2_duty_init(PWMV2_Type *ptr,
                     uint32_t PWM_PRD,
                     uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE,
                     uint8_t CMP_PWM_REGISTER_UPDATE_TYPE,
                     uint8_t CMP_SOURCE) {
}

void bldc_foc_pwmset(BLDC_CONTROL_PWMOUT_PARA *par) {
}

void adc_pins_init(void) {
}

void adc_init_udc_temp(ADC16_Type *ptr, uint8_t udc_channel, uint8_t temp_channel, uint32_t sample_cycle) {
}

void adc_cfg_init(ADC16_Type *ptr, uint8_t channel, uint32_t sample_cycle, uint32_t ADC_MODULE, uint32_t ADC_TRG) {
}

void init_trigger_mux(TRGM_Type *ptr, uint8_t TRAG_INPUT, uint8_t TRAG_INPUT_FOR_ADC) {
}

void init_trigger_cfg(
    ADC16_Type *ptr, uint8_t trig_ch, uint8_t channel, bool inten, uint32_t ADC_MODULE, uint8_t ADC_PREEMPT_TRIG_LEN) {
}

void adc_module_cfg(adc_type *adc_typ, uint8_t adc_module, ADC16_Type *adc_base_ptr) {
}

void pwmv2_trigfor_adc_init(PWMV2_Type *ptr,
                            uint32_t PWM_PRD,
                            uint32_t PWM_CNT,
                            uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE,
                            uint8_t CMP_PWM_REGISTER_UPDATE_TYPE,
                            uint8_t PWM_TRIGOUT_CH_ADC,
                            uint8_t CMP_SOURCE,
                            uint8_t PWM_CH_TRIG_ADC) {
}

void pwmv2_trigfor_sei_init(PWMV2_Type *ptr,
                            uint32_t PWM_PRD,
                            uint32_t PWM_CNT,
                            uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE,
                            uint8_t CMP_PWM_REGISTER_UPDATE_TYPE,
                            uint8_t PWM_TRIGOUT_CH_SEI,
                            uint8_t CMP_SOURCE,
                            uint8_t PWM_CH_TRIG_SEI) {
}

void pwm_ccr_set(uint32_t ccr1, uint32_t ccr2, uint32_t ccr3) {
    /* 写入TIM1三相PWM比较寄存器（CH1/CH2/CH3） */
    TIM1->CCR1 = ccr1;
    TIM1->CCR2 = ccr2;
    TIM1->CCR3 = ccr3;
}

void adc_isr_enable(void) {
}

void isr_adc(void) {
}

uint32_t motor_encoder_spi(uint8_t in_out) {
    return 0;
}

uint64_t get_clock_cpu_ms(void) {
    return 0;
}

uint8_t get_ver_id(void) {
    return 0;
}

extern ControllerStruct controller_eyou;

void dbg_cmd_set(void) {
    char *loc;
    char *token;

    if (usart_rx_len == 0) return;

    if (NULL != strstr((const char *)dbgRecvBuf, "logid")) {
        loc        = strstr((char *)dbgRecvBuf, "logid");
        token      = strtok(loc, "logid");
        dbgLogFlag = atoi((char *)token);
        printf("logid:%d\r\n", dbgLogFlag);
    }
    if (NULL != strstr((const char *)dbgRecvBuf, "logtest")) {
        loc         = strstr((char *)dbgRecvBuf, "logtest");
        token       = strtok(loc, "logtest");
        testLogFlag = atoi((char *)token);
        printf("logtest:%d\r\n", testLogFlag);
    }
    if (NULL != strstr((const char *)dbgRecvBuf, "logfreq")) {
        loc        = strstr((char *)dbgRecvBuf, "logfreq");
        token      = strtok(loc, "logfreq");
        logPriodMs = atoi((char *)token);
        printf("logfreq:%d\r\n", logPriodMs);
    }

    /* 带宽测试命令: bwtest1=电流环 (保守版: 10-1500Hz, inject 0.3A, bias 0.5A) */
    if (NULL != strstr((const char *)dbgRecvBuf, "bwtest")) {
        loc = strstr((char *)dbgRecvBuf, "bwtest");
        token = strtok(loc, "bwtest");
        int which = atoi((char *)token);
        printf("bwtest:%d\r\n", which);
        if (which == 1) {
            TestCurrentLoopBandwidth();
        } else if (which == 2) {
            TestSpeedLoopBandwidth();
        } else if (which == 3) {
            TestMotorParamsIdent();
        } else if (which == 4) {
            TestFluxIdent();
        } else if (which == 5) {
            TestInertiaIdent();
        } else if (which == 6) {
            TestAutoTuneCurrent();
        } else if (which == 7) {
            TestAutoTuneSpeed();
        } else if (which == 8) {
            TestAutoTunePosition();
        } else if (which == 9) {
            TestPositionLoopBandwidth();
        }
    }

    if (NULL != strstr((char *)dbgRecvBuf, "CurrentPID")) {
        printf("CurrentPID1:%d, %d, %d\r\n",
               controller_eyou.IncPID_QAxis.P,
               controller_eyou.IncPID_QAxis.I,
               controller_eyou.IncPID_QAxis.D);
        loc = strstr((char *)dbgRecvBuf, "Kp");
        if (loc != NULL) {
            token          = strtok(loc, "Kp");
            uint32_t Data0 = atoi(token);
            loc            = strstr((char *)dbgRecvBuf, "Ki");
            token          = strtok(loc, "Ki");
            uint32_t Data1 = atoi(token);
            loc            = strstr((char *)dbgRecvBuf, "Kd");
            token          = strtok(loc, "Kd");
            uint32_t Data2 = atoi(token);

            controller_eyou.IncPID_QAxis.P = Data0; controller_eyou.FlashData.Current_Kp = Data0;
            controller_eyou.IncPID_QAxis.I = Data1; controller_eyou.FlashData.Current_Ki = Data1;
            controller_eyou.IncPID_QAxis.D = Data2; controller_eyou.FlashData.Current_Kd = Data2;
            controller_eyou.IncPID_DAxis.P = Data0;
            controller_eyou.IncPID_DAxis.I = Data1;
            controller_eyou.IncPID_DAxis.D = Data2;
            printf("CurrentPID2:%d, %d, %d\r\n",
                   controller_eyou.IncPID_QAxis.P,
                   controller_eyou.IncPID_QAxis.I,
                   controller_eyou.IncPID_QAxis.D);
        }
    }

    if (NULL != strstr((char *)dbgRecvBuf, "SpeedPID")) {
        printf("SpeedPID:%d, %d, %d\r\n",
               controller_eyou.IncPID_Speed.P,
               controller_eyou.IncPID_Speed.I,
               controller_eyou.IncPID_Speed.D);
        loc = strstr((char *)dbgRecvBuf, "Kp");
        if (loc != NULL) {
            token          = strtok(loc, "Kp");
            uint32_t Data0 = atoi(token);
            loc            = strstr((char *)dbgRecvBuf, "Ki");
            token          = strtok(loc, "Ki");
            uint32_t Data1 = atoi(token);
            loc            = strstr((char *)dbgRecvBuf, "Kd");
            token          = strtok(loc, "Kd");
            uint32_t Data2 = atoi(token);

            controller_eyou.IncPID_Speed.P = Data0; controller_eyou.FlashData.Speed_Kp = Data0;
            controller_eyou.IncPID_Speed.I = Data1; controller_eyou.FlashData.Speed_Ki = Data1;
            controller_eyou.IncPID_Speed.D = Data2; controller_eyou.FlashData.Speed_Kd = Data2;
            printf("SpeedPID:%d, %d, %d\r\n",
                   controller_eyou.IncPID_Speed.P,
                   controller_eyou.IncPID_Speed.I,
                   controller_eyou.IncPID_Speed.D);
        }
    }

    if (NULL != strstr((char *)dbgRecvBuf, "PositionPID")) {
        printf("PositionPID:%d, %d, %d\r\n",
               controller_eyou.IncPID_Position.P,
               controller_eyou.IncPID_Position.I,
               controller_eyou.IncPID_Position.D);
        loc = strstr((char *)dbgRecvBuf, "Kp");
        if (loc != NULL) {
            token          = strtok(loc, "Kp");
            uint32_t Data0 = atoi(token);
            loc            = strstr((char *)dbgRecvBuf, "Ki");
            token          = strtok(loc, "Ki");
            uint32_t Data1 = atoi(token);
            loc            = strstr((char *)dbgRecvBuf, "Kd");
            token          = strtok(loc, "Kd");
            uint32_t Data2 = atoi(token);

            controller_eyou.IncPID_Position.P = Data0; controller_eyou.FlashData.Position_Kp = Data0;
            controller_eyou.IncPID_Position.I = Data1; controller_eyou.FlashData.Position_Ki = Data1;
            controller_eyou.IncPID_Position.D = Data2; controller_eyou.FlashData.Position_Kd = Data2;
            printf("PositionPID:%d, %d, %d\r\n",
                   controller_eyou.IncPID_Position.P,
                   controller_eyou.IncPID_Position.I,
                   controller_eyou.IncPID_Position.D);
        }
    }

    /* injectV<mV>: 在 theta=0 注入指定 V_d (毫伏)，持续 5 秒，每 100ms 打印 I_a / I_d
       用法: injectV2000  → V_d=2.0V
       配合万用表测 a 相对中性点（或 a-b 线电压）验证 SVPWM 标度 */
    if (NULL != strstr((char *)dbgRecvBuf, "injectV")) {
        loc        = strstr((char *)dbgRecvBuf, "injectV");
        token      = strtok(loc, "injectV");
        int32_t mv = atoi((char *)token);
        float v_d  = mv / 1000.0f;

        printf("inject test: V_d=%.3fV, theta=0, duration=5s\r\n", v_d);

        uint8_t old_run = controller_eyou.foc_run;
        controller_eyou.foc_run = 1;
        controller_eyou.ident_test.enable = 1;
        controller_eyou.ident_test.amplitude = 0;
        controller_eyou.ident_test.settle_samples = 0;
        controller_eyou.ident_test.measure_samples = 0xFFFFFFFF;
        controller_eyou.ident_test.sample_count = 0;
        controller_eyou.V_d = (int32_t)(v_d * 1024);
        controller_eyou.V_q = 0;
        controller_eyou.theta_elec = 0;

        for (int i = 0; i < 50; i++) {
            HAL_Delay(100);
            int32_t i_a_q10 = controller_eyou.I_a;
            int32_t i_d_q10 = controller_eyou.I_d;
            float i_a_amp = i_a_q10 / 1024.0f;
            float i_d_amp = i_d_q10 / 1024.0f;
            float r_a = (fabsf(i_a_amp) > 0.01f) ? (v_d / fabsf(i_a_amp)) : 0.0f;
            float r_d = (fabsf(i_d_amp) > 0.01f) ? (v_d / fabsf(i_d_amp)) : 0.0f;
            printf("[%2d] I_a=%6.3fA  I_d=%6.3fA  R(via Ia)=%.4fOhm  R(via Id)=%.4fOhm  Udc=%lu  CCR1=%lu CCR2=%lu CCR3=%lu\r\n",
                   i, i_a_amp, i_d_amp, r_a, r_d,
                   (unsigned long)motorProValue.Udc,
                   (unsigned long)TIM1->CCR1, (unsigned long)TIM1->CCR2, (unsigned long)TIM1->CCR3);
        }

        controller_eyou.ident_test.enable = 0;
        controller_eyou.V_d = 0;
        controller_eyou.V_q = 0;
        set_phase_voltage(&controller_eyou, 0, 0, 0);
        controller_eyou.foc_run = old_run;
        printf("inject test done\r\n");
    }

    /* Cali: 电角度偏置辨识 + 擦 Flash + 重新写入
       流程同 PHU: ElecAngleEstimate → Flash_EraseSector → WriteDataToFlash */
    if (NULL != strstr((char *)dbgRecvBuf, "Cali")) {
        ElecAngleEstimate(&controller_eyou);
        if (Flash_EraseSector() != HAL_OK) {
            printf("Cali: Flash erase FAIL\r\n");
        } else {
            WriteDataToFlash();
            printf("Cali done\r\n");
        }
    }

    if (NULL != strstr((char *)dbgRecvBuf, "Run")) {
        loc                     = strstr((char *)dbgRecvBuf, "cmd");
        token                   = strtok(loc, "cmd");
        controller_eyou.foc_run = atoi(token);

        loc                             = strstr((char *)dbgRecvBuf, "M");
        token                           = strtok(loc, "M");
        controller_eyou.controller_mode = atoi(token);
        loc                             = strstr((char *)dbgRecvBuf, "tar");
        token                           = strtok(loc, "tar");
        int32_t Data                    = atoi(token);

        if (controller_eyou.controller_mode == PROFILE_TORQUE_MODE ||
            controller_eyou.controller_mode == CYCLIC_SYNC_TORQUE_MODE) {
            int32_t iq = Data;
            if (iq > INC_PID_SPEED_LIMIT) { iq = INC_PID_SPEED_LIMIT; printf("  iq cmd clamped to +%d\r\n", INC_PID_SPEED_LIMIT); }
            else if (iq < -INC_PID_SPEED_LIMIT) { iq = -INC_PID_SPEED_LIMIT; printf("  iq cmd clamped to -%d\r\n", INC_PID_SPEED_LIMIT); }
            controller_eyou.I_q_ref = iq;
            controller_eyou.velocity_ref = 0;
        } else if (controller_eyou.controller_mode == PROFILE_VELOCITY_MOCE ||
                   controller_eyou.controller_mode == CYCLIC_SYNC_VELOCITY_MODE) {
            controller_eyou.velocity_ref = Data * 1024 * 25;
        } else if (controller_eyou.controller_mode == PROFILE_POSITION_MODE ||
                   controller_eyou.controller_mode == CYCLIC_SYNC_TORQUE_MODE) {
            int32_t p_ref = Data * 1024;
            if (controller_eyou.FlashData.PositionLimitFlag == 50) {
                int32_t pmax = controller_eyou.FlashData.MaxPositionLimit;
                int32_t pmin = controller_eyou.FlashData.MinPositionLimit;
                if (p_ref > pmax) { p_ref = pmax; printf("  pos cmd clamped to MaxPos=%ld\r\n", (long)pmax); }
                else if (p_ref < pmin) { p_ref = pmin; printf("  pos cmd clamped to MinPos=%ld\r\n", (long)pmin); }
            }
            controller_eyou.position_ref = p_ref;
        }
        printf("run mod_Target: %d, %d\r\n", controller_eyou.controller_mode, Data);
    }

    /* enable<0/1>: PWM 使能/失能控制
       用法: enable1 → 使能 PWM 输出
             enable0 → 失能 PWM 输出 */
    if (NULL != strstr((char *)dbgRecvBuf, "enable")) {
        loc   = strstr((char *)dbgRecvBuf, "enable");
        token = strtok(loc, "enable");
        int en = atoi(token);

        if (en) {
            controller_eyou.I_q_ref = 0;
            controller_eyou.velocity_ref = 0;
            controller_eyou.position_ref = controller_eyou.real_position_out;
            controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
            controller_eyou.foc_run = 2;
            TIM1->CCER |= 0x0555u;
            printf("PWM enabled, mode=Torque, I_q_ref=0 (CCER=0x%04X)\r\n", (unsigned int)TIM1->CCER);
        } else {
            controller_eyou.I_q_ref = 0;
            controller_eyou.velocity_ref = 0;
            controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
            TIM1->CCER &= ~0x0555u;
            controller_eyou.foc_run = 0;
            printf("PWM disabled, foc_run=0 (CCER=0x%04X)\r\n", (unsigned int)TIM1->CCER);
        }
    }

    /* canrxdbg<0/1>: 开启/关闭 CAN 收帧调试打印 */
    if (NULL != strstr((char *)dbgRecvBuf, "canrxdbg")) {
        loc = strstr((char *)dbgRecvBuf, "canrxdbg");
        token = strtok(loc, "canrxdbg");
        g_can_rx_debug = (uint8_t)atoi((char *)token);
        printf("CAN RX debug: %s\r\n", g_can_rx_debug ? "ON" : "OFF");
    }

    /* canstat: 打印 FDCAN 状态 + 重置 TX FIFO (恢复 Bus-Off / FIFO 卡死) */
    if (NULL != strstr((char *)dbgRecvBuf, "canstat")) {
        FDCAN_ProtocolStatusTypeDef ps;
        FDCAN_ErrorCountersTypeDef ec;
        HAL_FDCAN_GetProtocolStatus(&hfdcan1, &ps);
        HAL_FDCAN_GetErrorCounters(&hfdcan1, &ec);
        uint32_t tx_free = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
        printf("FDCAN: TxErr=%lu RxErr=%lu BusOff=%lu ErrPassive=%lu Warning=%lu\r\n",
               (unsigned long)ec.TxErrorCnt, (unsigned long)ec.RxErrorCnt,
               (unsigned long)ps.BusOff, (unsigned long)ps.ErrorPassive,
               (unsigned long)ps.Warning);
        printf("  LastErr=%lu DataLastErr=%lu Activity=%lu\r\n",
               (unsigned long)ps.LastErrorCode, (unsigned long)ps.DataLastErrorCode,
               (unsigned long)ps.Activity);
        printf("  TxFifoFree=%lu/10  TxFailCnt=%lu  NodeID=%u  AutoReport=%u\r\n",
               (unsigned long)tx_free,
               (unsigned long)can_wly_get_tx_fail_count(),
               can_wly_get_node_id(), 0);
        /* 如果 BusOff 或 FIFO 满, 复位外设恢复 */
        if (ps.BusOff || tx_free == 0) {
            printf("  -> reset FDCAN peripheral\r\n");
            HAL_FDCAN_Stop(&hfdcan1);
            HAL_FDCAN_Start(&hfdcan1);
            printf("  reset done\r\n");
        }
    }

    /* cantest<N>: CAN 协议单元自测 (不接总线, 模拟收帧 → 打印内部状态) */
    if (NULL != strstr((char *)dbgRecvBuf, "cantest")) {
        loc = strstr((char *)dbgRecvBuf, "cantest");
        token = strtok(loc, "cantest");
        int tc = atoi((char *)token);
        printf("=== cantest%d ===\r\n", tc);
        g_cantest_stub = 1;

        switch (tc) {
        case 1: {
            /* 0x200 速度指令: v_raw=36205(=20rpm output), ID=1 */
            uint8_t d[] = {0x6D, 0x8D, 0x01};
            printf("  [RX] ID=0x200 D=6D 8D 01 (v_raw=36205 -> 20rpm, ID=1)\r\n");
            fdcan_rx_user(0x200, d, sizeof(d));
            float v_rad_s = (float)controller_eyou.velocity_ref / (1024.0f * 25.0f) * (2.0f * 3.14159265f / 60.0f);
            printf("  velocity_ref=%d (internal)\r\n", (int)controller_eyou.velocity_ref);
            printf("  -> output rad/s=%.4f (expect ~2.0944)\r\n", v_rad_s);
            printf("  mode=%d (expect %d=PROFILE_VELOCITY)\r\n",
                   controller_eyou.controller_mode, PROFILE_VELOCITY_MOCE);
            break;
        }
        case 2: {
            /* 0x400 位置指令: p_raw=中点(0x800000), v_raw=0x8000, ID=1 */
            uint8_t d[] = {0x00, 0x00, 0x80, 0x00, 0x80, 0x01};
            uint32_t p_raw = 0x800000;
            uint16_t v_raw = 0x8000;
            printf("  [RX] ID=0x400 D=00 00 80 00 80 01 (p_raw=0x%06X, v_raw=0x%04X, ID=1)\r\n",
                   (unsigned int)p_raw, (unsigned int)v_raw);
            fdcan_rx_user(0x400, d, sizeof(d));
            printf("  position_ref=%d (1deg/1024 LSB)\r\n", (int)controller_eyou.position_ref);
            printf("  Pid_PositionLimit=%d\r\n", (int)controller_eyou.FlashData.Pid_PositionLimit);
            printf("  mode=%d (expect %d=PROFILE_POSITION)\r\n",
                   controller_eyou.controller_mode, PROFILE_POSITION_MODE);
            break;
        }
        case 3: {
            /* 直接调 pack_status_frame 打印 12 字节 → 验证速度刻度疑点 */
            printf("  Current state: pos_out=%d, dtheta_mech_out=%d, I_q=%d\r\n",
                   (int)controller_eyou.real_position_out,
                   (int)controller_eyou.dtheta_mech_out,
                   (int)controller_eyou.I_q);
            /* 触发状态帧发送 (stub 模式会打印 hex) */
            fdcan_rx_user(0x080, NULL, 0);
            printf("  [Decode hint] pos: float_to_uint(pos_rad, %.1f, %.1f, 24)\r\n",
                   g_can_wly_lim.pos_min, g_can_wly_lim.pos_max);
            printf("  [Decode hint] vel: float_to_uint(vel_rad_s, %.1f, %.1f, 16)\r\n",
                   g_can_wly_lim.spd_min, g_can_wly_lim.spd_max);
            break;
        }
        case 4: {
            /* SDO 读 0x2000 (pos_min) */
            uint8_t d[] = {0x40, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
            printf("  [RX] ID=0x601 SDO Read idx=0x2000 (pos_min)\r\n");
            fdcan_rx_user(0x600 + can_wly_get_node_id(), d, 8);
            printf("  expect: pos_min=%.3f rad\r\n", g_can_wly_lim.pos_min);
            break;
        }
        case 5: {
            /* SDO 写 0x2000 (pos_min = -10.0f) */
            union { float f; uint8_t b[4]; } cv;
            cv.f = -10.0f;
            uint8_t d[] = {0x23, 0x00, 0x20, 0x00, cv.b[0], cv.b[1], cv.b[2], cv.b[3]};
            printf("  [RX] ID=0x601 SDO Write idx=0x2000 val=-10.0f\r\n");
            float old_val = g_can_wly_lim.pos_min;
            fdcan_rx_user(0x600 + can_wly_get_node_id(), d, 8);
            printf("  pos_min: %.3f -> %.3f (expect -10.000)\r\n", old_val, g_can_wly_lim.pos_min);
            /* 恢复原值 */
            g_can_wly_lim.pos_min = old_val;
            printf("  (restored to %.3f)\r\n", g_can_wly_lim.pos_min);
            break;
        }
        case 6: {
            /* 0x701 控制帧使能: D[0..6]=0xFF, D[7]=0xFA */
            uint8_t d[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFA};
            uint8_t old_run = controller_eyou.foc_run;
            printf("  [RX] ID=0x701 CTRL_ENABLE (D[7]=0xFA)\r\n");
            fdcan_rx_user(0x700 + can_wly_get_node_id(), d, 8);
            printf("  foc_run: %d -> %d (expect 1)\r\n", old_run, controller_eyou.foc_run);
            controller_eyou.foc_run = old_run;
            printf("  (restored foc_run=%d)\r\n", controller_eyou.foc_run);
            break;
        }
        case 7: {
            /* 超时保护测试: 注入一帧激活看门狗, 然后等 250ms 看 CommunicateErr */
            uint8_t d[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
            printf("  [RX] ID=0x701 CLR_ERR -> activate timeout watchdog\r\n");
            controller_eyou.ServoErrFlag.All_Flag = 0;
            fdcan_rx_user(0x700 + can_wly_get_node_id(), d, 8);
            printf("  Watchdog armed. Waiting 250ms (no frames)...\r\n");
            g_cantest_stub = 0;
            for (int i = 0; i < 250; i++) {
                HAL_Delay(1);
                can_wly_tick_1ms();
            }
            g_cantest_stub = 1;
            printf("  CommunicateErr=%d (expect 1)\r\n",
                   (int)controller_eyou.ServoErrFlag.Bit.CommunicateErr);
            controller_eyou.ServoErrFlag.All_Flag = 0;
            break;
        }
        case 8: {
            /* 0x500 MIT 指令: 12 字节 */
            /* p_raw=中点, v_raw=中点, t_raw=中点, kp_raw=0x4000, kd_raw=0x4000, ID=1 */
            uint8_t d[] = {0x00, 0x00, 0x80,   /* POS[23:0] = 0x800000 */
                           0x00, 0x80,          /* VEL[15:0] = 0x8000 */
                           0x00, 0x80,          /* T[15:0]   = 0x8000 */
                           0x00, 0x40,          /* Kp[15:0]  = 0x4000 */
                           0x00, 0x40,          /* Kd[15:0]  = 0x4000 */
                           0x01};               /* CANID     = 1 */
            printf("  [RX] ID=0x500 MIT 12B: p=mid, v=mid, t=mid, Kp=0x4000, Kd=0x4000\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  position_ref=%d, I_q_ref=%d\r\n",
                   (int)controller_eyou.position_ref, (int)controller_eyou.I_q_ref);
            printf("  mode=%d (expect %d=CSP)\r\n",
                   controller_eyou.controller_mode, CYCLIC_SYNC_POSITION_MODE);
            printf("  NOTE: Kp/Kd discarded in current impl (see疑点#4)\r\n");
            break;
        }
        default:
            printf("  Unknown cantest%d (valid: 1-8)\r\n", tc);
            break;
        }
        g_cantest_stub = 0;
        printf("=== cantest%d done ===\r\n", tc);
    }

    memset((uint8_t *)dbgRecvBuf, 0, usart_rx_len);
    usart_rx_len = 0;
}

void dbg_log_print(void) {
    switch (dbgLogFlag) {
    case 1:
        controller_eyou.velocity_ref = 0;
        printf("dbg_log_print test\r\n");
        dbgLogFlag = 0;
        break;
    case 10:
        printf("Angle_elec_360: %d, %d, %d, %d, %d\r\n",
               controller_eyou.now_mechposition,
               controller_eyou.theta_elec,
               controller_eyou.real_position_out,
               controller_eyou.real_position,
               controller_eyou.dtheta_mech / 1024);
        break;
    case 11: {
        /* 输出端编码器调试：检查 inner_raw 是否更新 */
        DPT_Angles angles;
        DPT_GetLatestAngles(&angles);
        printf("Out_enc: inner_raw=%lu outer_raw=%lu | pos_out=%ld pos_out_pre=%ld | old_cnt=%ld circle=%d\r\n",
               (unsigned long)angles.inner_raw,
               (unsigned long)angles.outer_raw,
               (long)controller_eyou.real_position_out,
               (long)controller_eyou.real_position_out_pre,
               (long)controller_eyou.old_angle_count_out,
               controller_eyou.circle_count_out);
        break;
    }
    case 30:
        printf("current_get: %d,%d\r\n", controller_eyou.V_q, controller_eyou.V_d);
        break;
    case 40:
        printf("current_pi: %d, %d, %d, %d, %d, %d, %d\r\n",
               controller_eyou.I_q,
               controller_eyou.I_d,
               controller_eyou.V_q,
               controller_eyou.V_d,
               controller_eyou.I_q_ref,
               controller_eyou.I_d_ref,
               controller_eyou.I_q_ref_filterd);
        break;
    case 50:
        printf("speed: %d, %d, %d, %d, %d\r\n",
               controller_eyou.velocity_ref / 1024,
               controller_eyou.velocity_ref_filterd / 1024,
               controller_eyou.dtheta_mech / 1024,
               controller_eyou.dtheta_mech_out / 1024,
               controller_eyou.dtheta_mech / 1024 - controller_eyou.dtheta_mech_out / 1024);
        break;
    case 60:
        printf("%d, %d, %d\r\n", controller_eyou.CCR2, controller_eyou.CCR3, controller_eyou.CCR4);
        break;
    case 70:
        printf("%d, %d, %d\r\n", controller_eyou.I_a, controller_eyou.I_b, controller_eyou.I_c);
        break;
    case 90:
        printf("%d, %d, %d\r\n", controller_eyou.Ia_raw, controller_eyou.Ib_raw, controller_eyou.Ic_raw);
        break;
    case 100:
        printf("position: %f, %f, %f, %d\r\n",
               controller_eyou.position_ref / 1024.0,
               controller_eyou.real_position_out / 1024.0,
               (controller_eyou.position_ref - controller_eyou.real_position_out) / 1024.0,
               controller_eyou.FlashData.mech_offest_out);
        break;
    case 110: {
        /* ADC ISR 分段耗时（us, 480MHz → 1us=480 cycles）
         * 格式 last/max, 单位 us
         * total = 整体 ISR 耗时
         * read  = 电流 raw 读取 + 校准结构更新
         * enc   = Encoder_data_Calculate + Encoder_out_data_Calculate
         * pos   = foc_position_close_loop (2.5kHz, 4 拍 1 次)
         * vel   = foc_velocity_close_loop (5kHz, 2 拍 1 次)
         * cur   = foc_current_close_loop + SVPWM (10kHz, 每拍) */
        uint32_t t_tot = g_adc_isr_cycles;
        uint32_t t_tot_max = g_adc_isr_cycles_max;
        uint32_t t_read = g_adc_isr_t_read, t_read_max = g_adc_isr_t_read_max;
        uint32_t t_enc  = g_adc_isr_t_enc,  t_enc_max  = g_adc_isr_t_enc_max;
        uint32_t t_pos  = g_adc_isr_t_pos,  t_pos_max  = g_adc_isr_t_pos_max;
        uint32_t t_vel  = g_adc_isr_t_vel,  t_vel_max  = g_adc_isr_t_vel_max;
        uint32_t t_cur  = g_adc_isr_t_cur,  t_cur_max  = g_adc_isr_t_cur_max;
        printf("adc_isr_us tot:%lu/%lu read:%lu/%lu enc:%lu/%lu pos:%lu/%lu vel:%lu/%lu cur:%lu/%lu\r\n",
               (unsigned long)(t_tot / 480),  (unsigned long)(t_tot_max / 480),
               (unsigned long)(t_read / 480), (unsigned long)(t_read_max / 480),
               (unsigned long)(t_enc / 480),  (unsigned long)(t_enc_max / 480),
               (unsigned long)(t_pos / 480),  (unsigned long)(t_pos_max / 480),
               (unsigned long)(t_vel / 480),  (unsigned long)(t_vel_max / 480),
               (unsigned long)(t_cur / 480),  (unsigned long)(t_cur_max / 480));
        break;
    }
    case 120: {
        /* 开环测试状态（每 1s 打印一次，屏蔽 logfreq 低值刷屏）
         * 原 main.c "OpenLoop" 调试块 */
        static uint32_t t120 = 0;
        uint32_t now = HAL_GetTick();
        if (now - t120 < 1000) break;
        t120 = now;
        printf("OpenLoop: theta=%u I_a=%d I_b=%d I_c=%d V_d=%d V_q=%d\r\n",
               controller_eyou.theta_elec,
               controller_eyou.I_a, controller_eyou.I_b, controller_eyou.I_c,
               v_d_test, v_q_test);
        break;
    }
    case 130: {
        /* DPT 编码器统计（触发频率/成功/跳过 + 最新角度，每 1s 打印）
         * DPT_GetAndResetStats 会清零累计量，依赖真实 1s 窗口 —— 不可用 logfreq 替代 */
        static uint32_t t130 = 0;
        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms = now - t130;
        if (elapsed_ms < 1000) break;
        t130 = now;

        uint32_t trig, succ, skip, last_us, min_us, max_us;
        DPT_GetAndResetStats(&trig, &succ, &skip, &last_us, &min_us, &max_us);

        DPT_Angles angles;
        DPT_GetLatestAngles(&angles);

        uint32_t freq_hz = (trig * 1000) / elapsed_ms;
        printf("Inner:%.2f Outer:%.2f Sta:0x%02X | Trig:%luHz Succ:%lu Skip:%lu | Enc_us last=%lu min=%lu max=%lu\r\n",
               angles.inner_deg, angles.outer_deg, angles.status,
               (unsigned long)freq_hz, (unsigned long)succ, (unsigned long)skip,
               (unsigned long)last_us, (unsigned long)min_us, (unsigned long)max_us);
        break;
    }
    case 140: {
        /* CC4 / Enc / ADC 相对时序（以 ADC ISR entry 为 0 时刻，每 1s）
         * 原 main.c "时序测试" 块 */
        static uint32_t t140 = 0;
        uint32_t now = HAL_GetTick();
        if (now - t140 < 1000) break;
        t140 = now;

        uint32_t t_cc4_in   = g_tim1_cc4_cycles;
        uint32_t t_cc4_out  = g_tim1_cc4_exit_cycles;
        uint32_t t_enc_done = g_tim1_enc_done_cycles;
        uint32_t t_adc_in   = g_adc_isr_in_cycles;
        uint32_t t_adc_out  = g_adc_isr_out_cycles;

        int32_t d_adc_out  = (int32_t)(t_adc_out  - t_adc_in) / 480;
        int32_t d_cc4_in   = (int32_t)(t_cc4_in   - t_adc_in) / 480;
        int32_t d_cc4_out  = (int32_t)(t_cc4_out  - t_adc_in) / 480;
        int32_t d_enc_done = (int32_t)(t_enc_done - t_adc_in) / 480;
        /* 编码器 44us 后完成，可能是上一周期完成时刻，负值加 100us 换算 */
        if (d_enc_done < 0) d_enc_done += 100;

        printf("T0=ADC_in | ADC_out=%+ldus CC4_in=%+ldus CC4_out=%+ldus Enc_done=%+ldus\r\n",
               (long)d_adc_out, (long)d_cc4_in, (long)d_cc4_out, (long)d_enc_done);
        break;
    }
    case 150: {
        /* ADC 注入采样速率 + 电流原始值 + 校准 offset + TIM1 完成时刻（每 1s）
         * 原 main.c "ADC注入采样检测" 块 */
        static uint32_t t150 = 0;
        static uint32_t last_sample_count = 0;
        uint32_t now = HAL_GetTick();
        uint32_t elapsed_ms = now - t150;
        if (elapsed_ms < 1000) break;
        t150 = now;

        uint32_t cnt_now = g_foc_current.sample_count;
        int32_t  ia      = g_foc_current.i_a_raw;
        int32_t  ib      = g_foc_current.i_b_raw;
        uint32_t t_done  = g_foc_current.tim1_done_cnt;
        int32_t  off_a   = g_adc_offset_a;
        int32_t  off_b   = g_adc_offset_b;

        uint32_t delta   = cnt_now - last_sample_count;
        last_sample_count = cnt_now;
        uint32_t rate_hz = (delta * 1000) / elapsed_ms;

        /* TIM1 CNT 480MHz / 2(中央对齐) → 240 ticks/us */
        printf("ADC Rate=%luHz Cnt=%lu | Ia=%ld Ib=%ld | OffA=%ld OffB=%ld | t_done=%lu.%luus\r\n",
               (unsigned long)rate_hz, (unsigned long)cnt_now, (long)ia, (long)ib,
               (long)off_a, (long)off_b,
               (unsigned long)(t_done / 240), (unsigned long)((t_done % 240) * 10 / 240));
        break;
    }
    case 151: {
        /* 温度 + 电压监控（每 1s，单位 0.1）
         * Udc: 0.1V, Temp: 0.1°C */
        static uint32_t t151 = 0;
        uint32_t now = HAL_GetTick();
        if (now - t151 < 1000) break;
        t151 = now;

        /* 调用 ADC 转换函数，更新 motorProValue */
        adc_convert();

        /* 获取转换后的值（已经是 0.1 单位）*/
        uint32_t udc_01v = motorProValue.Udc;           /* 0.1V */
        int8_t t_board_c = motorProValue.board_temp;    /* °C */
        int8_t t_motor_c = motorProValue.motor_temp;    /* °C */

        /* 转换为 0.1 单位 */
        int16_t t_board_01c = t_board_c * 10;          /* 0.1°C */
        int16_t t_motor_01c = t_motor_c * 10;          /* 0.1°C */

        printf("Udc=%lu(0.1V) Tboard=%d(0.1C) Tmotor=%d(0.1C) | Raw: VDC=%lu Tmos=%lu Tmot=%lu\r\n",
               (unsigned long)udc_01v,
               (int)t_board_01c,
               (int)t_motor_01c,
               (unsigned long)g_vdc_raw,
               (unsigned long)g_temp_mos_raw,
               (unsigned long)g_temp_motor_raw);
        break;
    }
    case 160:
        /* 写Flash：把当前FlashData保存 */
        WriteDataToFlash();
        printf("WriteDataToFlash\r\n");
        dbgLogFlag = 0;
        break;
    case 161:
        /* 擦除Flash扇区：下次上电会触发版本不匹配重新初始化 */
        if (Flash_EraseSector() == HAL_OK) {
            printf("Flash erase OK\r\n");
        } else {
            printf("Flash erase FAIL\r\n");
        }
        dbgLogFlag = 0;
        break;
    case 162: {
        /* Dump FlashData：打印RAM和Flash中的数据，便于对比 */
        FlashSavedData flash_copy;
        Flash_ReadData(FLASH_USER_START_ADDR, &flash_copy, sizeof(FlashSavedData));

        FlashSavedData *ram = &controller_eyou.FlashData;
        FlashSavedData *fls = &flash_copy;

        printf("===== FlashData Dump (RAM vs Flash) =====\r\n");
        printf("                    RAM              Flash\r\n");
        printf("[Header]\r\n");
        printf("  Ver             %-16u %u\r\n", ram->StructVersion, fls->StructVersion);
        printf("  CurFlag         0x%02X             0x%02X\r\n", ram->CurrentFlag, fls->CurrentFlag);
        printf("  AngFlag         0x%02X             0x%02X\r\n", ram->AngleOffsetFlag, fls->AngleOffsetFlag);
        printf("  PidFlag         0x%02X             0x%02X\r\n", ram->PidFlag, fls->PidFlag);
        printf("  ArrFlag         0x%02X             0x%02X\r\n", ram->ArrivedFlag, fls->ArrivedFlag);
        printf("  RunFlag         0x%02X             0x%02X\r\n", ram->RunDataFlag, fls->RunDataFlag);
        printf("  PosLimFlag      0x%02X             0x%02X\r\n", ram->PositionLimitFlag, fls->PositionLimitFlag);
        printf("  PrtFlag         0x%02X             0x%02X\r\n", ram->ProteckKeyFlag, fls->ProteckKeyFlag);
        printf("[Iofs]\r\n");
        printf("  Ia              %-16u %u\r\n", ram->Ia_offset, fls->Ia_offset);
        printf("  Ib              %-16u %u\r\n", ram->Ib_offset, fls->Ib_offset);
        printf("  Ic              %-16u %u\r\n", ram->Ic_offset, fls->Ic_offset);
        printf("[Angle]\r\n");
        printf("  elec            %-16u %u\r\n", ram->elec_offset, fls->elec_offset);
        printf("  PhaseOrder      %-16u %u\r\n", ram->PhaseOrder, fls->PhaseOrder);
        printf("  mech            %-16ld %ld\r\n", (long)ram->mech_offest, (long)fls->mech_offest);
        printf("  mech_out        %-16ld %ld\r\n", (long)ram->mech_offest_out, (long)fls->mech_offest_out);
        printf("[PosPID]\r\n");
        printf("  Kp              %-16lu %lu\r\n", (unsigned long)ram->Position_Kp, (unsigned long)fls->Position_Kp);
        printf("  Ki              %-16lu %lu\r\n", (unsigned long)ram->Position_Ki, (unsigned long)fls->Position_Ki);
        printf("  Kd              %-16lu %lu\r\n", (unsigned long)ram->Position_Kd, (unsigned long)fls->Position_Kd);
        printf("  Lim             %-16ld %ld\r\n", (long)ram->Pid_PositionLimit, (long)fls->Pid_PositionLimit);
        printf("  FF_Kp           %-16ld %ld\r\n", (long)ram->PosErrFF_Kp, (long)fls->PosErrFF_Kp);
        printf("[SpdPID]\r\n");
        printf("  Kp              %-16lu %lu\r\n", (unsigned long)ram->Speed_Kp, (unsigned long)fls->Speed_Kp);
        printf("  Ki              %-16lu %lu\r\n", (unsigned long)ram->Speed_Ki, (unsigned long)fls->Speed_Ki);
        printf("  Kd              %-16lu %lu\r\n", (unsigned long)ram->Speed_Kd, (unsigned long)fls->Speed_Kd);
        printf("  Lim             %-16ld %ld\r\n", (long)ram->Pid_SpeedLimit, (long)fls->Pid_SpeedLimit);
        printf("[CurPID]\r\n");
        printf("  Kp              %-16lu %lu\r\n", (unsigned long)ram->Current_Kp, (unsigned long)fls->Current_Kp);
        printf("  Ki              %-16lu %lu\r\n", (unsigned long)ram->Current_Ki, (unsigned long)fls->Current_Ki);
        printf("  Kd              %-16lu %lu\r\n", (unsigned long)ram->Current_Kd, (unsigned long)fls->Current_Kd);
        printf("  Lim             %-16ld %ld\r\n", (long)ram->Pid_CurrentLimit, (long)fls->Pid_CurrentLimit);
        printf("[Arrive]\r\n");
        printf("  Pos(0.1d)       %-16u %u\r\n", ram->PositionArrivedValue, fls->PositionArrivedValue);
        printf("  Spd(0.1rpm)     %-16u %u\r\n", ram->SpeedArrivedValue, fls->SpeedArrivedValue);
        printf("  Cur(0.1A)       %-16u %u\r\n", ram->CurrentArrivedValue, fls->CurrentArrivedValue);
        printf("[Run]\r\n");
        printf("  Mode            %-16u %u\r\n", ram->RunMode, fls->RunMode);
        printf("  MaxSpd(0.1rpm)  %-16ld %ld\r\n", (long)ram->MaxSpeed, (long)fls->MaxSpeed);
        printf("  MaxCur(0.1A)    %-16u %u\r\n", ram->MaxCurrent, fls->MaxCurrent);
        printf("  PosMax          %-16ld %ld\r\n", (long)ram->MaxPositionLimit, (long)fls->MaxPositionLimit);
        printf("  PosMin          %-16ld %ld\r\n", (long)ram->MinPositionLimit, (long)fls->MinPositionLimit);
        printf("[Prtct]\r\n");
        printf("  Sto1            %-16u %u\r\n", ram->Sto_1_protectKey, fls->Sto_1_protectKey);
        printf("  Sto2            %-16u %u\r\n", ram->Sto_2_protectKey, fls->Sto_2_protectKey);
        printf("  BusVol          %-16u %u\r\n", ram->BusVolProteckKey, fls->BusVolProteckKey);
        printf("  LockRot         %-16u %u\r\n", ram->LockedRotorProtectKey, fls->LockedRotorProtectKey);
        printf("  StoState        %-16lu %lu\r\n", (unsigned long)ram->stoStateFlag, (unsigned long)fls->stoStateFlag);
        printf("[Misc]\r\n");
        printf("  BrakeT          %-16u %u\r\n", ram->brake_time, fls->brake_time);
        printf("  Crc             0x%08lX       0x%08lX\r\n", (unsigned long)ram->Crc, (unsigned long)fls->Crc);
        printf("[Size] sizeof(FlashSavedData)=%u\r\n", (unsigned)sizeof(FlashSavedData));
        printf("===== End =====\r\n");
        dbgLogFlag = 0;
        break;
    }
    case 163:
        /* 清除所有故障标志 */
        ClearFaults(1);
        printf("All faults cleared, ready to restart\r\n");
        dbgLogFlag = 0;
        break;
    case 164:
        /* 设置当前位置为零点 (等效 PHU HOMING_MODE 置零) */
        controller_eyou.controller_mode = HOMING_MODE;
        Reset_objReset_Output_Encoder(1);
        controller_eyou.UserDataSaveFlag = 1;
        printf("Reset_objReset_Output_Encoder: %ld\r\n",
               (long)controller_eyou.FlashData.mech_offest_out);
        Reset_objReset_Output_Encoder(0);
        dbgLogFlag = 0;
        break;
    default:
        break;
    }
}
