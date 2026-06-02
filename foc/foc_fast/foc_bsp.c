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
#include "ota_app.h"
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

/* 方向相关相位补偿:
 *   theta_comp = offset × 18.2 counts + dtheta × comp / 10
 *   offset: 固定偏置 (×0.1°), dbg: offsetpos/offsetneg
 *   comp:   速度相关 (×0.1 倍 dtheta), dbg: comppos/compneg
 * 默认: offset=-12°/−68°(正/反转100rpm调好), comp=0(预留) */
int16_t g_theta_offset_pos = 0;//-120;   /* -12.0° */
int16_t g_theta_offset_neg = 0;//-680;   /* -68.0° */
int16_t g_theta_comp_pos = 20;//23;
int16_t g_theta_comp_neg = 26;//23;

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

    /* version: 打印固件/硬件版本 + 编译时间, 用于核对烧录的 hex */
    if (NULL != strstr((const char *)dbgRecvBuf, "version")) {
        printf(FW_BANNER_FMT, SOFT_VERSION, HARD_VERSION, BUILD_DATE, BUILD_TIME);
    }

    /* reset: 触发 NVIC 系统复位, 用于上位机重启 MCU 而无需按物理按键 */
    if (NULL != strstr((const char *)dbgRecvBuf, "reset")) {
        printf("System reset requested...\r\n");
        /* 给 USART DMA 把上一行字节送出, 否则上位机收不到 banner */
        HAL_Delay(20);
        fault_safe_shutdown();
        HAL_Delay(50);
        NVIC_SystemReset();
    }

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
        } else if (which == 10) {
            TestDeadtimeCalibration();
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
       流程同 PHU: ElecAngleEstimate → Flash_EraseSector → WriteDataToFlash
       注意：必须先停机 (foc_run=0) 避免 ISR 覆盖开环 PWM
       故障后 MOE/CCER 已被关闭, 需要重新使能才能输出 PWM */
    if (NULL != strstr((char *)dbgRecvBuf, "Cali")) {
        uint8_t old_run = controller_eyou.foc_run;
        controller_eyou.foc_run = 0;   /* 停机，禁用 ISR 闭环 */
        controller_eyou.ServoErrFlag.All_Flag = 0;  /* 清故障标志，防止 1ms tick 再次关 PWM */
        HAL_Delay(10);                  /* 等 ISR 退出 */

        /* 重新使能 PWM 输出 (故障停机后 MOE=0, CCER=0) */
        TIM1->CCER |= 0x0555u;         /* 使能 CH1/CH2/CH3 输出 */
        __HAL_TIM_MOE_ENABLE(&htim1);   /* 使能主输出 */

        ElecAngleEstimate(&controller_eyou);

        if (Flash_EraseSector() != HAL_OK) {
            printf("Cali: Flash erase FAIL\r\n");
        } else {
            WriteDataToFlash();
            printf("Cali done\r\n");
        }

        controller_eyou.foc_run = old_run;
    }

    if (NULL != strstr((char *)dbgRecvBuf, "Run")) {
        loc                     = strstr((char *)dbgRecvBuf, "cmd");
        token                   = strtok(loc, "cmd");
        int cmd_val             = atoi(token);

        loc                             = strstr((char *)dbgRecvBuf, "M");
        token                           = strtok(loc, "M");
        int mode_val                    = atoi(token);

        /* tar 参数统一用浮点解析，避免 strtok 破坏字符串 */
        char *tar_str = strstr((char *)dbgRecvBuf, "tar");
        float tar_value = (tar_str != NULL) ? atof(tar_str + 3) : 0.0f;

        /* cmd=0 视为停机命令, 走主动刹车流程, 不直接 foc_run=0 (会让 PWM 卡死在上次 CCR) */
        if (cmd_val == 0) {
            fault_safe_shutdown();
            printf("Runcmd0: safe shutdown initiated\r\n");
            memset((uint8_t *)dbgRecvBuf, 0, usart_rx_len);
            usart_rx_len = 0;
            return;
        }

        /* 启动前检查: 刹车进行中 / 故障未清 → 静默拒绝 (不打印, 避免上位机周期发命令时刷屏)
         * 用户要清错请发 logid163 */
        if (fault_brake_is_active()) return;
        if (controller_eyou.ServoErrFlag.All_Flag != 0) return;

        /* 区分"首次启动"和"运行中改目标":
         * 已在跑且模式不变 → 只改目标值, 不重置 PID (避免积分器清零导致抖动)
         * 首次启动或切模式 → 完整重置 */
        uint8_t already_running = (controller_eyou.foc_run >= 1 &&
                                   controller_eyou.controller_mode == mode_val);
        if (!already_running) {
            ResetControlData(&controller_eyou);
            controller_eyou.foc_run         = cmd_val;
            controller_eyou.controller_mode = mode_val;
            TIM1->BDTR |= TIM_BDTR_MOE;
            TIM1->CCER |= 0x0555u;
        }

        if (controller_eyou.controller_mode == PROFILE_TORQUE_MODE ||
            controller_eyou.controller_mode == CYCLIC_SYNC_TORQUE_MODE) {
            /* tar 字段语义: N·m (输出端扭矩), 经 Kt LUT 反查到 q 轴电流 Q10 */
            float tq_nm = tar_value;
            int32_t iq = (int32_t)(can_wly_Nm_to_iA(tq_nm) * 1024.0f);
            int32_t max_cur = (int32_t)controller_eyou.FlashData.MaxCurrent;
            if (iq >  max_cur) { iq =  max_cur; printf("  iq cmd clamped to +%d\r\n", max_cur); }
            else if (iq < -max_cur) { iq = -max_cur; printf("  iq cmd clamped to -%d\r\n", max_cur); }
            controller_eyou.I_q_ref = iq;
            controller_eyou.velocity_ref = 0;
            printf("  tar=%.3f Nm -> Iq=%ld Q10\r\n", tq_nm, (long)iq);
            printf("run mod_Target: %d, %.3f Nm\r\n", controller_eyou.controller_mode, tq_nm);
        } else if (controller_eyou.controller_mode == PROFILE_VELOCITY_MOCE ||
                   controller_eyou.controller_mode == CYCLIC_SYNC_VELOCITY_MODE) {
            float vel_rpm = tar_value;
            controller_eyou.velocity_ref = (int32_t)(vel_rpm * 1024.0f * 25.0f);
            printf("run mod_Target: %d, %.2f rpm\r\n", controller_eyou.controller_mode, vel_rpm);
        } else if (controller_eyou.controller_mode == PROFILE_POSITION_MODE ||
                   controller_eyou.controller_mode == CYCLIC_SYNC_POSITION_MODE) {
            float pos_deg = tar_value;
            int32_t p_ref = (int32_t)(pos_deg * 1024.0f);
            if (controller_eyou.FlashData.PositionLimitFlag == 50) {
                int32_t pmax = controller_eyou.FlashData.MaxPositionLimit;
                int32_t pmin = controller_eyou.FlashData.MinPositionLimit;
                float pmax_deg = pmax / 1024.0f;
                float pmin_deg = pmin / 1024.0f;
                if (pos_deg > pmax_deg) {
                    p_ref = pmax;
                    pos_deg = pmax_deg;
                    printf("  pos cmd clamped to MaxPos=%.2f deg\r\n", pmax_deg);
                } else if (pos_deg < pmin_deg) {
                    p_ref = pmin;
                    pos_deg = pmin_deg;
                    printf("  pos cmd clamped to MinPos=%.2f deg\r\n", pmin_deg);
                }
            }
            controller_eyou.position_ref = p_ref;
            printf("run mod_Target: %d, %.2f deg\r\n", controller_eyou.controller_mode, pos_deg);
        } else {
            printf("run mod_Target: %d, %.2f\r\n", controller_eyou.controller_mode, tar_value);
        }
    }

    /* enable<0/1>: PWM 使能/失能控制
       用法: enable1 → 使能 PWM 输出
             enable0 → 失能 PWM 输出 */
    if (NULL != strstr((char *)dbgRecvBuf, "enable")) {
        loc   = strstr((char *)dbgRecvBuf, "enable");
        token = strtok(loc, "enable");
        int en = atoi(token);

        if (en) {
            /* 启动前检查: 刹车进行中 / 故障未清 → 拒绝启动 */
            if (fault_brake_is_active()) {
                printf("enable: brake in progress, ignored\r\n");
            } else if (controller_eyou.ServoErrFlag.All_Flag != 0) {
                printf("enable: fault active (0x%08lX), clear first\r\n",
                       (unsigned long)controller_eyou.ServoErrFlag.All_Flag);
            } else {
                /* 重置 PID 积分, 避免历史残留导致首拍喷大扭矩 */
                ResetControlData(&controller_eyou);
                controller_eyou.I_q_ref = 0;
                controller_eyou.velocity_ref = 0;
                controller_eyou.position_ref = controller_eyou.real_position_out;
                controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
                controller_eyou.foc_run = 2;
                /* 恢复 MOE (上次刹车结束时清掉了) + 打开 PWM 通道 */
                TIM1->BDTR |= TIM_BDTR_MOE;
                TIM1->CCER |= 0x0555u;
                printf("PWM enabled, mode=Torque, I_q_ref=0 (CCER=0x%04X)\r\n", (unsigned int)TIM1->CCER);
            }
        } else {
            /* 安全停机：对齐 PHU 方案（零电压滑行 → 低速刹车 → 高阻） */
            fault_safe_shutdown();
            printf("PWM disable requested, safe shutdown initiated\r\n");
        }
    }

    /* canrxdbg<0/1>: 开启/关闭 CAN 收帧调试打印 */
    if (NULL != strstr((char *)dbgRecvBuf, "canrxdbg")) {
        loc = strstr((char *)dbgRecvBuf, "canrxdbg");
        token = strtok(loc, "canrxdbg");
        g_can_rx_debug = (uint8_t)atoi((char *)token);
        printf("CAN RX debug: %s\r\n", g_can_rx_debug ? "ON" : "OFF");
    }

    /* offsetpos<N>: 设置正转固定角度偏置 (单位 ×0.1°, 例 400=40°)
       注: 用 atoi(loc+len) 而不是 strtok, 避免破坏 buffer 影响后续命令解析
       (上位机一次发 4 行 phase comp 命令时, strtok 会把后续命令首字母改成 \0) */
    if (NULL != (loc = strstr((char *)dbgRecvBuf, "offsetpos"))) {
        g_theta_offset_pos = (int16_t)atoi(loc + strlen("offsetpos"));
        printf("offset_pos=%d (×0.1°) = %.1f deg\r\n",
               g_theta_offset_pos, g_theta_offset_pos * 0.1f);
    }

    /* offsetneg<N>: 设置反转固定角度偏置 (单位 ×0.1°) */
    if (NULL != (loc = strstr((char *)dbgRecvBuf, "offsetneg"))) {
        g_theta_offset_neg = (int16_t)atoi(loc + strlen("offsetneg"));
        printf("offset_neg=%d (×0.1°) = %.1f deg\r\n",
               g_theta_offset_neg, g_theta_offset_neg * 0.1f);
    }

    /* comppos<N>: 正转速度相关补偿系数 (×0.1 倍 dtheta) */
    if (NULL != (loc = strstr((char *)dbgRecvBuf, "comppos"))) {
        g_theta_comp_pos = (int16_t)atoi(loc + strlen("comppos"));
        printf("comp_pos=%d (×0.1)\r\n", g_theta_comp_pos);
    }

    /* compneg<N>: 反转速度相关补偿系数 (×0.1 倍 dtheta) */
    if (NULL != (loc = strstr((char *)dbgRecvBuf, "compneg"))) {
        g_theta_comp_neg = (int16_t)atoi(loc + strlen("compneg"));
        printf("comp_neg=%d (×0.1)\r\n", g_theta_comp_neg);
    }

    /* savephasecomp: 保存相位补偿参数到 Flash */
    if (NULL != strstr((char *)dbgRecvBuf, "savephasecomp")) {
        extern void SavePhaseCompToFlash(void);
        SavePhaseCompToFlash();
    }

    /* getparams: 查询所有 PID + 相位补偿参数 (上位机自动读取用)
     * 输出格式 (一行一参数, key=value, 上位机正则匹配):
     *   PARAMS_BEGIN
     *   CurKp=45 CurKi=4 CurKd=0
     *   SpdKp=1500 SpdKi=10 SpdKd=0
     *   PosKp=3016 PosKi=9 PosKd=0
     *   OffPos=0 OffNeg=0 CompPos=20 CompNeg=26
     *   PARAMS_END
     */
    if (NULL != strstr((char *)dbgRecvBuf, "getparams")) {
        printf("PARAMS_BEGIN\r\n");
        printf("CurKp=%u CurKi=%u CurKd=%u\r\n",
               (unsigned)controller_eyou.IncPID_QAxis.P,
               (unsigned)controller_eyou.IncPID_QAxis.I,
               (unsigned)controller_eyou.IncPID_QAxis.D);
        printf("SpdKp=%u SpdKi=%u SpdKd=%u\r\n",
               (unsigned)controller_eyou.IncPID_Speed.P,
               (unsigned)controller_eyou.IncPID_Speed.I,
               (unsigned)controller_eyou.IncPID_Speed.D);
        printf("PosKp=%u PosKi=%u PosKd=%u\r\n",
               (unsigned)controller_eyou.IncPID_Position.P,
               (unsigned)controller_eyou.IncPID_Position.I,
               (unsigned)controller_eyou.IncPID_Position.D);
        printf("OffPos=%d OffNeg=%d CompPos=%d CompNeg=%d\r\n",
               g_theta_offset_pos, g_theta_offset_neg,
               g_theta_comp_pos, g_theta_comp_neg);
        printf("PARAMS_END\r\n");
    }

    /* testfreq<N>: 设置单频注入频率 (Hz, 等效 CAN 0x2F06) */
    extern uint32_t can_wly_get_test_freq(void);
    extern void can_wly_set_test_freq(uint32_t hz);
    if (NULL != strstr((char *)dbgRecvBuf, "testfreq")) {
        loc = strstr((char *)dbgRecvBuf, "testfreq");
        token = strtok(loc, "testfreq");
        uint32_t hz = (uint32_t)atoi((char *)token);
        can_wly_set_test_freq(hz);
        printf("test_freq=%u Hz\r\n", can_wly_get_test_freq());
    }

    /* testampl<N>: 设置单频注入幅值 (Q10, 等效 CAN 0x2F07, 1024=1A) */
    extern uint32_t can_wly_get_test_ampl(void);
    extern void can_wly_set_test_ampl(uint32_t q10);
    if (NULL != strstr((char *)dbgRecvBuf, "testampl")) {
        loc = strstr((char *)dbgRecvBuf, "testampl");
        token = strtok(loc, "testampl");
        uint32_t q10 = (uint32_t)atoi((char *)token);
        can_wly_set_test_ampl(q10);
        printf("test_ampl=%u Q10 (%.2f A)\r\n",
               can_wly_get_test_ampl(), can_wly_get_test_ampl() / 1024.0f);
    }

    /* teststart: 启动单频注入 + 0x7FD 数据流 (等效 CAN 0x2F05 cmd=1) */
    extern void can_wly_test_start(void);
    if (NULL != strstr((char *)dbgRecvBuf, "teststart")) {
        can_wly_test_start();
        printf("test started: %u Hz, %.2f A\r\n",
               can_wly_get_test_freq(), can_wly_get_test_ampl() / 1024.0f);
    }

    /* teststop: 停止单频注入 (等效 CAN 0x2F05 cmd=0) */
    extern void can_wly_test_stop(void);
    extern uint32_t can_wly_get_test_tx_ok(void);
    extern uint32_t can_wly_get_test_tx_fail(void);
    if (NULL != strstr((char *)dbgRecvBuf, "teststop")) {
        uint32_t ok = can_wly_get_test_tx_ok();
        uint32_t fail = can_wly_get_test_tx_fail();
        can_wly_test_stop();
        printf("test stopped: 0x7FD tx_ok=%u, tx_fail=%u\r\n",
               (unsigned)ok, (unsigned)fail);
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
            printf("  Current state: pos_out=%d, dtheta_mech_out_eq=%d, I_q=%d\r\n",
                   (int)controller_eyou.real_position_out,
                   (int)(controller_eyou.dtheta_mech / 25),
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
            /* 0x500 MIT 指令: 12 字节 - 基础测试（中点值 + 中等 Kp/Kd）*/
            /* p_raw=中点, v_raw=中点, t_raw=中点, kp_raw=0x4000, kd_raw=0x4000, ID=1 */
            uint8_t d[] = {0x00, 0x00, 0x80,   /* POS[23:0] = 0x800000 */
                           0x00, 0x80,          /* VEL[15:0] = 0x8000 */
                           0x00, 0x80,          /* T[15:0]   = 0x8000 */
                           0x00, 0x40,          /* Kp[15:0]  = 0x4000 */
                           0x00, 0x40,          /* Kd[15:0]  = 0x4000 */
                           0x01};               /* CANID     = 1 */
            printf("  [RX] ID=0x500 MIT 12B: p=mid, v=mid, t=mid, Kp=0x4000, Kd=0x4000\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_p_des=%.4f rad, mit_v_des=%.4f rad/s, mit_t_ff=%.3f A\r\n",
                   controller_eyou.mit_p_des, controller_eyou.mit_v_des, controller_eyou.mit_t_ff);
            printf("  mit_kp=%.2f, mit_kd=%.2f\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            printf("  mode=%d (expect %d=MIT_PD_MODE)\r\n",
                   controller_eyou.controller_mode, MIT_PD_MODE);
            break;
        }
        case 9: {
            /* MIT 测试 - 最大 Kp (500) + 零位置/速度 */
            uint8_t d[] = {0x00, 0x00, 0x80,   /* POS = 0 rad (中点) */
                           0x00, 0x80,          /* VEL = 0 rad/s (中点) */
                           0x00, 0x80,          /* T = 0 N·m (中点) */
                           0xFF, 0xFF,          /* Kp = 500 (最大) */
                           0x00, 0x00,          /* Kd = 0 (最小) */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=0, v=0, t=0, Kp=MAX(500), Kd=0\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_kp=%.2f (expect 500.00), mit_kd=%.2f (expect 0.00)\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            printf("  mode=%d\r\n", controller_eyou.controller_mode);
            break;
        }
        case 10: {
            /* MIT 测试 - 最大 Kd (20) + 零 Kp */
            uint8_t d[] = {0x00, 0x00, 0x80,
                           0x00, 0x80,
                           0x00, 0x80,
                           0x00, 0x00,          /* Kp = 0 */
                           0xFF, 0xFF,          /* Kd = 20 (最大) */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=0, v=0, t=0, Kp=0, Kd=MAX(20)\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_kp=%.2f (expect 0.00), mit_kd=%.2f (expect 20.00)\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            printf("  mode=%d\r\n", controller_eyou.controller_mode);
            break;
        }
        case 11: {
            /* MIT 测试 - 正向位置 (+3.5 rad ≈ +200°) + 正速度 (+10 rad/s) */
            /* pos_min=-7, pos_max=7 → 中点=0, +3.5 rad = 0x800000 + 0x800000*0.5 = 0xC00000 */
            uint8_t d[] = {0x00, 0x00, 0xC0,   /* POS = +3.5 rad (75% 量程) */
                           0x00, 0xC0,          /* VEL = +10 rad/s (75% 量程) */
                           0x00, 0x80,          /* T = 0 */
                           0x99, 0x19,          /* Kp = 50 (10% 量程) */
                           0xA3, 0x30,          /* Kd = 3.8 (19% 量程, 0x30A3) */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=+3.5rad, v=+10rad/s, t=0, Kp=50, Kd=3.8\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_p_des=%.4f rad (expect ~3.5), mit_v_des=%.4f rad/s (expect ~10.0)\r\n",
                   controller_eyou.mit_p_des, controller_eyou.mit_v_des);
            printf("  mit_kp=%.2f, mit_kd=%.2f\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            break;
        }
        case 12: {
            /* MIT 测试 - 负向位置 (-3.5 rad) + 负速度 (-10 rad/s) */
            uint8_t d[] = {0x00, 0x00, 0x40,   /* POS = -3.5 rad (25% 量程) */
                           0x00, 0x40,          /* VEL = -10 rad/s (25% 量程) */
                           0x00, 0x80,          /* T = 0 */
                           0x99, 0x19,          /* Kp = 50 */
                           0xA3, 0x30,          /* Kd = 3.8 (0x30A3) */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=-3.5rad, v=-10rad/s, t=0, Kp=50, Kd=3.8\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_p_des=%.4f rad (expect ~-3.5), mit_v_des=%.4f rad/s (expect ~-10.0)\r\n",
                   controller_eyou.mit_p_des, controller_eyou.mit_v_des);
            printf("  mit_kp=%.2f, mit_kd=%.2f\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            break;
        }
        case 13: {
            /* MIT 测试 - 最大正扭矩前馈 (+500 N·m) */
            uint8_t d[] = {0x00, 0x00, 0x80,
                           0x00, 0x80,
                           0xFF, 0xFF,          /* T = +500 N·m (最大) */
                           0x00, 0x00,          /* Kp = 0 (纯前馈) */
                           0x00, 0x00,          /* Kd = 0 */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=0, v=0, t=+500Nm(MAX), Kp=0, Kd=0\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_t_ff=%.3f A (expect ~168, LUT 100A->260Nm 之外线性外推)\r\n",
                   controller_eyou.mit_t_ff);
            printf("  mit_kp=%.2f, mit_kd=%.2f (both expect 0.00)\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            break;
        }
        case 14: {
            /* MIT 测试 - 最大负扭矩前馈 (-500 N·m) */
            uint8_t d[] = {0x00, 0x00, 0x80,
                           0x00, 0x80,
                           0x00, 0x00,          /* T = -500 N·m (最小) */
                           0x00, 0x00,
                           0x00, 0x00,
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=0, v=0, t=-500Nm(MIN), Kp=0, Kd=0\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_t_ff=%.3f A (expect ~-168, LUT 外推下限)\r\n",
                   controller_eyou.mit_t_ff);
            break;
        }
        case 15: {
            /* MIT 测试 - 边界位置 (pos_max = +7 rad) */
            uint8_t d[] = {0xFF, 0xFF, 0xFF,   /* POS = +7 rad (最大) */
                           0x00, 0x80,
                           0x00, 0x80,
                           0x00, 0x40,          /* Kp = 125 (25% 量程, 0x4000) */
                           0x00, 0x80,          /* Kd = 10 (50% 量程, 0x8000) */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=+7rad(MAX), v=0, t=0, Kp=125, Kd=10\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_p_des=%.4f rad (expect ~7.0)\r\n", controller_eyou.mit_p_des);
            printf("  mit_kp=%.2f, mit_kd=%.2f\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            break;
        }
        case 16: {
            /* MIT 测试 - 边界位置 (pos_min = -7 rad) */
            uint8_t d[] = {0x00, 0x00, 0x00,   /* POS = -7 rad (最小) */
                           0x00, 0x80,
                           0x00, 0x80,
                           0x00, 0x40,          /* Kp = 125 (0x4000) */
                           0x00, 0x80,          /* Kd = 10 (0x8000) */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=-7rad(MIN), v=0, t=0, Kp=125, Kd=10\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_p_des=%.4f rad (expect ~-7.0)\r\n", controller_eyou.mit_p_des);
            printf("  mit_kp=%.2f, mit_kd=%.2f\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            break;
        }
        case 17: {
            /* MIT 测试 - 边界速度 (spd_max = +20 rad/s) */
            uint8_t d[] = {0x00, 0x00, 0x80,
                           0xFF, 0xFF,          /* VEL = +20 rad/s (最大) */
                           0x00, 0x80,
                           0x00, 0x20,          /* Kp = 62.5 (0x2000) */
                           0xFF, 0xFF,          /* Kd = 20 (最大) */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=0, v=+20rad/s(MAX), t=0, Kp=62.5, Kd=20\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_v_des=%.4f rad/s (expect ~20.0)\r\n", controller_eyou.mit_v_des);
            printf("  mit_kd=%.2f (expect 20.00)\r\n", controller_eyou.mit_kd);
            break;
        }
        case 18: {
            /* MIT 测试 - 边界速度 (spd_min = -20 rad/s) */
            uint8_t d[] = {0x00, 0x00, 0x80,
                           0x00, 0x00,          /* VEL = -20 rad/s (最小) */
                           0x00, 0x80,
                           0x00, 0x20,          /* Kp = 62.5 (0x2000) */
                           0xFF, 0xFF,          /* Kd = 20 */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=0, v=-20rad/s(MIN), t=0, Kp=62.5, Kd=20\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_v_des=%.4f rad/s (expect ~-20.0)\r\n", controller_eyou.mit_v_des);
            printf("  mit_kd=%.2f (expect 20.00)\r\n", controller_eyou.mit_kd);
            break;
        }
        case 19: {
            /* MIT 测试 - 典型阻抗控制参数 (Kp=100, Kd=5, 小扭矩偏置) */
            uint8_t d[] = {0x00, 0x00, 0x80,   /* POS = 0 */
                           0x00, 0x80,          /* VEL = 0 */
                           0x8F, 0x82,          /* T = +10 N·m (52% 量程, 0x828F) */
                           0x33, 0x33,          /* Kp = 100 (20% 量程) */
                           0x00, 0x40,          /* Kd = 5 (25% 量程, 0x4000) */
                           0x01};
            printf("  [RX] ID=0x500 MIT: p=0, v=0, t=+10Nm, Kp=100, Kd=5 (typical impedance)\r\n");
            fdcan_rx_user(0x500, d, 12);
            printf("  mit_t_ff=%.3f A\r\n", controller_eyou.mit_t_ff);
            printf("  mit_kp=%.2f (expect ~100), mit_kd=%.2f (expect ~5)\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            break;
        }
        default:
            printf("  Unknown cantest%d (valid: 1-19)\r\n", tc);
            break;
        }
        g_cantest_stub = 0;
        printf("=== cantest%d done ===\r\n", tc);
    }

    /* mit<N>: MIT 模式实测序列 (0x500 CAN-FD 帧, 需主站 ≤10ms 重发)
     * 用法: 串口发 "mit0" ~ "mit7" 模拟 Step 0~7, 每步持续 100ms 后自动退出
     * Step 0: 安全态准备 (失能→清错→置零→使能)
     * Step 1~7: MIT 指令 (切模式/位置保持/阶跃/提刚度/速度跟随/转矩前馈) */
    if (NULL != strstr((char *)dbgRecvBuf, "mit")) {
        loc = strstr((char *)dbgRecvBuf, "mit");
        token = strtok(loc, "mit");
        int step = atoi((char *)token);
        printf("=== mit%d ===\r\n", step);
        g_cantest_stub = 1;

        switch (step) {
        case 0: {
            /* Step 0: 安全态准备 (0x701 经典 CAN, 每帧间隔 20ms) */
            uint8_t d_disable[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
            uint8_t d_clear[]   = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
            uint8_t d_zero[]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
            uint8_t d_enable[]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFA};

            printf("  [0.1] 失能\r\n");
            fdcan_rx_user(0x701, d_disable, 8);
            HAL_Delay(20);

            printf("  [0.2] 清错\r\n");
            fdcan_rx_user(0x701, d_clear, 8);
            HAL_Delay(20);

            printf("  [0.3] 置零 (当前位置对齐 0 rad)\r\n");
            fdcan_rx_user(0x701, d_zero, 8);
            HAL_Delay(20);

            printf("  [0.4] 使能\r\n");
            fdcan_rx_user(0x701, d_enable, 8);
            printf("  foc_run=%d (expect 2)\r\n", controller_eyou.foc_run);
            break;
        }
        case 1: {
            /* Step 1: 切模式不动作 (p=0, v=0, t=0, Kp=0, Kd=0) */
            uint8_t d[] = {0xFF, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x01};
            printf("  [RX] ID=0x500 DLC=12 (p=0, v=0, t=0, Kp=0, Kd=0)\r\n");
            for (int i = 0; i < 10; i++) {
                fdcan_rx_user(0x500, d, 12);
                HAL_Delay(10);
            }
            printf("  mode=%d (expect %d=MIT_PD_MODE)\r\n",
                   controller_eyou.controller_mode, MIT_PD_MODE);
            printf("  I_q=%d (expect ~0)\r\n", controller_eyou.I_q);
            break;
        }
        case 2: {
            /* Step 2: 位置保持 (p=0, v=0, t=0, Kp=10, Kd=1) */
            uint8_t d[] = {0xFF, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0x1E, 0x05, 0xCC, 0x0C, 0x01};
            printf("  [RX] ID=0x500 DLC=12 (p=0, Kp=10, Kd=1)\r\n");
            printf("  手扭输出端应有回弹力，松手回 0 位 ±0.5°\r\n");
            for (int i = 0; i < 10; i++) {
                fdcan_rx_user(0x500, d, 12);
                HAL_Delay(10);
            }
            printf("  mit_kp=%.2f, mit_kd=%.2f\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            break;
        }
        case 3: {
            /* Step 3: 阶跃 +5° (p=+0.0873 rad, Kp=10, Kd=1) */
            uint8_t d[] = {0xB2, 0x9A, 0x81, 0xFF, 0x7F, 0xFF, 0x7F, 0x1E, 0x05, 0xCC, 0x0C, 0x01};
            printf("  [RX] ID=0x500 DLC=12 (p=+0.0873 rad = +5°, Kp=10, Kd=1)\r\n");
            printf("  输出端应转 +5°，超调 ≤20%%，稳态误差 ≤0.5°\r\n");
            for (int i = 0; i < 10; i++) {
                fdcan_rx_user(0x500, d, 12);
                HAL_Delay(10);
            }
            printf("  mit_p_des=%.4f rad (expect 0.0873)\r\n", controller_eyou.mit_p_des);
            break;
        }
        case 4: {
            /* Step 4: 阶跃 −5° (p=−0.0873 rad, Kp=10, Kd=1) */
            uint8_t d[] = {0x4C, 0x65, 0x7E, 0xFF, 0x7F, 0xFF, 0x7F, 0x1E, 0x05, 0xCC, 0x0C, 0x01};
            printf("  [RX] ID=0x500 DLC=12 (p=−0.0873 rad = −5°, Kp=10, Kd=1)\r\n");
            printf("  输出端应转 −5°\r\n");
            for (int i = 0; i < 10; i++) {
                fdcan_rx_user(0x500, d, 12);
                HAL_Delay(10);
            }
            printf("  mit_p_des=%.4f rad (expect -0.0873)\r\n", controller_eyou.mit_p_des);
            break;
        }
        case 5: {
            /* Step 5: 提刚度 +5° (p=+0.0873 rad, Kp=50, Kd=2) */
            uint8_t d[] = {0xB2, 0x9A, 0x81, 0xFF, 0x7F, 0xFF, 0x7F, 0x99, 0x19, 0x99, 0x19, 0x01};
            printf("  [RX] ID=0x500 DLC=12 (p=+0.0873 rad, Kp=50, Kd=2)\r\n");
            printf("  响应更快、刚度更高；如出现啸叫立刻停止\r\n");
            for (int i = 0; i < 10; i++) {
                fdcan_rx_user(0x500, d, 12);
                HAL_Delay(10);
            }
            printf("  mit_kp=%.2f (expect 50), mit_kd=%.2f (expect 2)\r\n",
                   controller_eyou.mit_kp, controller_eyou.mit_kd);
            break;
        }
        case 6: {
            /* Step 6: 速度跟随 (p=0, v=+0.5 rad/s, Kp=0, Kd=2) */
            uint8_t d[] = {0xFF, 0xFF, 0x7F, 0x32, 0x83, 0xFF, 0x7F, 0x00, 0x00, 0x99, 0x19, 0x01};
            printf("  [RX] ID=0x500 DLC=12 (v=+0.5 rad/s, Kp=0, Kd=2)\r\n");
            printf("  电机应以 ~0.5 rad/s ≈ 4.77 rpm 输出端转动\r\n");
            for (int i = 0; i < 10; i++) {
                fdcan_rx_user(0x500, d, 12);
                HAL_Delay(10);
            }
            printf("  mit_v_des=%.4f rad/s (expect 0.5)\r\n", controller_eyou.mit_v_des);
            break;
        }
        case 7: {
            /* Step 7: 转矩前馈 (t=+0.2 N·m, Kp=0, Kd=0) ⚠️ 无速度环保护 */
            uint8_t d[] = {0xFF, 0xFF, 0x7F, 0xFF, 0x7F, 0x19, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01};
            printf("  [RX] ID=0x500 DLC=12 (t=+0.2 N·m, Kp=0, Kd=0)\r\n");
            printf("  ⚠️ 转矩模式无速度环保护，空载会持续加速\r\n");
            printf("  看到电流稳定后立即发 mit0 或 mit1 停止\r\n");
            for (int i = 0; i < 10; i++) {
                fdcan_rx_user(0x500, d, 12);
                HAL_Delay(10);
            }
            printf("  mit_t_ff=%.2f A (expect ~0.2)\r\n", controller_eyou.mit_t_ff / 1024.0f);
            printf("  I_q=%d\r\n", controller_eyou.I_q);
            break;
        }
        default:
            printf("  Unknown mit%d (valid: 0-7)\r\n", step);
            break;
        }

        g_cantest_stub = 0;
        printf("=== mit%d done ===\r\n", step);
    }

    /* ---------------- OTA firmware upgrade (Stage 1) ----------------
       otabegin SIZE=<n> CRC=0x<hex> VER=<v>  → erase App-B, switch RX to OTA mode
       otaend                                 → finalize, verify CRC, write header
       otaabort                               → cancel, clear App-B header
       otaswap                                → reset (Stage 2 bootloader will pick) */
    extern volatile uint8_t g_ota_rx_mode;

    if (NULL != strstr((char *)dbgRecvBuf, "otabegin")) {
        char *p_size = strstr((char *)dbgRecvBuf, "SIZE=");
        char *p_crc  = strstr((char *)dbgRecvBuf, "CRC=0x");
        char *p_ver  = strstr((char *)dbgRecvBuf, "VER=");
        if (p_size && p_crc) {
            uint32_t size = (uint32_t)atoi(p_size + 5);
            uint32_t crc  = (uint32_t)strtoul(p_crc + 6, NULL, 16);
            uint32_t ver  = p_ver ? (uint32_t)atoi(p_ver + 4) : 0;
            if (ota_begin(size, crc, ver) == 0) {
                /* Switch RX path to OTA mode AFTER OTA_READY is printed,
                   so further bytes go to the OTA ring buffer. */
                g_ota_rx_mode = 1;
            }
        } else {
            printf("OTA_ERR bad_begin_args\r\n");
        }
    }
    else if (NULL != strstr((char *)dbgRecvBuf, "otaend")) {
        /* Caller flushes RX path back to text mode before sending otaend.
           If we reach here, g_ota_rx_mode has already been cleared by main loop. */
        ota_end();
    }
    else if (NULL != strstr((char *)dbgRecvBuf, "otaabort")) {
        g_ota_rx_mode = 0;
        ota_abort();
    }
    else if (NULL != strstr((char *)dbgRecvBuf, "otaswap")) {
        printf("Reboot for swap...\r\n");
        HAL_Delay(50);
        NVIC_SystemReset();
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
        /* I_q 用对外上报的 LPF 值 (~200Hz fc), 避免 PWM 纹波抽样混叠;
         * 控制环与诊断瞬时值仍走原始 controller_eyou.I_q */
        printf("current_pi: %d, %d, %d, %d, %d, %d, %d\r\n",
               can_wly_iq_fb_get(),
               controller_eyou.I_d,
               controller_eyou.V_q,
               controller_eyou.V_d,
               controller_eyou.I_q_ref,
               controller_eyou.I_d_ref,
               controller_eyou.I_q_ref_filterd);
        break;
    case 50:
        /* 列1=指令(rpm), 列2=斜坡后指令(rpm), 列3=电机端速度(rpm),
           列4=电机端折算到载端(rpm, =列3/25), 列5=指令-反馈(rpm) */
        printf("speed: %d, %d, %d, %d, %d\r\n",
               controller_eyou.velocity_ref / 1024,
               controller_eyou.velocity_ref_filterd / 1024,
               controller_eyou.dtheta_mech / 1024,
               (controller_eyou.dtheta_mech / 1024) / 25,
               (controller_eyou.velocity_ref - controller_eyou.dtheta_mech) / 1024);
        break;
    case 60:
        printf("%d, %d, %d\r\n", controller_eyou.CCR2, controller_eyou.CCR3, controller_eyou.CCR4);
        break;
    case 70:
        printf("%d, %d, %d, %d, %d, %d\r\n", controller_eyou.CCR2, controller_eyou.CCR3, controller_eyou.CCR4, controller_eyou.I_a, controller_eyou.I_b, controller_eyou.I_c);
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
        int16_t t_board_c = motorProValue.board_temp;   /* °C */
        int16_t t_motor_c = motorProValue.motor_temp;   /* °C */

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
        printf("[PhaseComp]\r\n");
        printf("  PhCompFlag      0x%02X             0x%02X\r\n",
               ram->PhaseCompFlag, fls->PhaseCompFlag);
        /* RAM 列显示当前生效的全局变量, Flash 列从 temp5/temp6 解包 */
        printf("  OffsetPos(0.1d) %-16d %d\r\n",
               (int)g_theta_offset_pos,
               (int)(int16_t)(fls->temp5 & 0xFFFF));
        printf("  OffsetNeg(0.1d) %-16d %d\r\n",
               (int)g_theta_offset_neg,
               (int)(int16_t)((fls->temp5 >> 16) & 0xFFFF));
        printf("  CompPos(0.1)    %-16d %d\r\n",
               (int)g_theta_comp_pos,
               (int)(int16_t)(fls->temp6 & 0xFFFF));
        printf("  CompNeg(0.1)    %-16d %d\r\n",
               (int)g_theta_comp_neg,
               (int)(int16_t)((fls->temp6 >> 16) & 0xFFFF));
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
    case 165:
        /* 查询当前故障标志 */
        printf("ServoErrFlag = 0x%08lX\r\n", (unsigned long)controller_eyou.ServoErrFlag.All_Flag);
        if (controller_eyou.ServoErrFlag.All_Flag == 0) {
            printf("  No faults\r\n");
        } else {
            extern void print_fault_types_pub(void);
            print_fault_types_pub();
        }
        dbgLogFlag = 0;
        break;
    case 200: {
        /* 扭矩诊断综合快照: 速度环 + 电流环 + 电压裕量 + BEMF + 限幅状态
         * 6 行块, 每行 [L200/n] 前缀便于 awk/grep 拆分.
         * 内部节流 50ms (20Hz), 不受 logfreq 影响 — 6 行 ~600B, 高速会撑爆 921600.
         *   行1 [L200/1]: 模式 / 故障 / 速度链 (load端 rpm)
         *   行2 [L200/2]: 速度PID 状态 (Iq_ref 来源)
         *   行3 [L200/3]: 电流PID + Vdq + Vs/Udc 调制度 + BEMF 估算
         *   行4 [L200/4]: 三相反馈 + αβ + 原始 ADC + theta_e
         *   行5 [L200/5]: PWM CCR 占空比 + 双环斜坡状态
         *   行6 [L200/6]: 外层限幅 + 输出端反馈 + 位置环饱和
         * 单位:
         *   spd_ref/spd_filt: load端 rpm (除以 25 后已是输出端 rpm)
         *   spd_mech: 电机端 rpm
         *   I_*: 0.01A (Q10 / 10.24 后保留两位)
         *   V_*: 0.01V (同上)
         */
        static uint32_t t200 = 0;
        uint32_t now200 = HAL_GetTick();
        if (now200 - t200 < 50) break;
        t200 = now200;

        ControllerStruct *c = &controller_eyou;

        /* 速度: velocity_ref/velocity_ref_filterd 单位 = 电机端 rpm × 1024, dtheta_mech 同单位
         * 打印转换为 load 端 0.01rpm: velocity_ref / (25 * 1024 / 100) */
        int32_t spd_ref_load_x100   = c->velocity_ref         / (25 * 1024 / 100);  /* 0.01 rpm load端 */
        int32_t spd_filt_load_x100  = c->velocity_ref_filterd / (25 * 1024 / 100);
        int32_t spd_mech_motor_x100 = c->dtheta_mech          / (1024 / 100);       /* 0.01 rpm 电机端 */
        int32_t spd_err_motor_x100  = (c->velocity_ref_filterd / 25 - c->dtheta_mech) / (1024 / 100);

        /* PID 饱和判定: |OutPut| 距离 OutputMax 不到 1 LSB 视为撞限 */
        int32_t spd_out      = c->IncPID_Speed.OutPut;
        int32_t spd_out_max  = c->IncPID_Speed.OutputMax;
        int sat_spd          = (spd_out >=  spd_out_max - 1) ? 1
                             : (spd_out <= -spd_out_max + 1) ? -1 : 0;

        int32_t iq_pid       = c->IncPID_QAxis.OutPut;        /* PI 自身输出 (BEMF FF 之前) */
        int32_t iq_pid_max   = c->IncPID_QAxis.OutputMax;
        int sat_iq           = (iq_pid >=  iq_pid_max - 1) ? 1
                             : (iq_pid <= -iq_pid_max + 1) ? -1 : 0;

        int32_t id_pid       = c->IncPID_DAxis.OutPut;
        int32_t id_pid_max   = c->IncPID_DAxis.OutputMax;
        int sat_id           = (id_pid >=  id_pid_max - 1) ? 1
                             : (id_pid <= -id_pid_max + 1) ? -1 : 0;

        /* Vs 矢量幅值 (Q10 V); g_vs_limit = Vdc/√3×1024 动态跟踪母线 */
        int32_t vs_q10       = (int32_t)sqrtf((float)c->V_d * c->V_d + (float)c->V_q * c->V_q);
        int sat_vs           = (vs_q10 >= g_vs_limit - 16) ? 1 : 0;

        /* 调制度 = Vs / (Udc/sqrt(3))(SVPWM 线性区上限). Udc 0.1V → V; */
        float udc_v          = (float)motorProValue.Udc * 0.1f;
        float vs_v           = (float)vs_q10 / 1024.0f;
        int   mod_pct        = (udc_v > 1.0f)
                             ? (int)(vs_v * 1.732f / udc_v * 100.0f) : 0;

        /* BEMF 估算 (即使 USE_BEMF_FF=0 也算出来给参考)
         *  ωe [rad/s] = dtheta_mech [rpm×1024] × bemf_omega_e_k (= NPP·2π/(1024·60))
         *  Vd_ff = -ωe × Lq × Iq
         *  Vq_ff = ωe × (Ld × Id + ψf)
         */
        float omega_e        = (float)c->dtheta_mech * c->bemf_omega_e_k;
        float vd_bemf_v      = -omega_e * c->ident_test.Lq * ((float)c->I_q / 1024.0f);
        float vq_bemf_v      = omega_e * (c->ident_test.Ld * ((float)c->I_d / 1024.0f)
                                         + c->ident_test.flux_psi);

        /* 输出 */
        printf("[L200/1] mode=%d run=%d err=0x%08lX | sref=%ld sflt=%ld smech=%ld serr=%ld (0.01rpm) | Imax=%u(Q10)\r\n",
               c->controller_mode, c->foc_run,
               (unsigned long)c->ServoErrFlag.All_Flag,
               (long)spd_ref_load_x100, (long)spd_filt_load_x100,
               (long)spd_mech_motor_x100, (long)spd_err_motor_x100,
               (unsigned)c->FlashData.MaxCurrent);

        printf("[L200/2] SpdPID Kp=%u Ki=%u Div=%u | aim=%ld now=%ld err=%ld out=%ld/%ld sat=%d -> Iq_ref=%ld(Q10)\r\n",
               (unsigned)c->IncPID_Speed.P, (unsigned)c->IncPID_Speed.I,
               (unsigned)c->IncPID_Speed.PID_Div,
               (long)c->IncPID_Speed.AimValue, (long)c->IncPID_Speed.NowValue,
               (long)c->IncPID_Speed.iError,
               (long)spd_out, (long)spd_out_max, sat_spd,
               (long)c->I_q_ref);

        printf("[L200/3] CurPID Kp=%u Ki=%u Div=%u | Iqref_f=%ld Iq=%ld Iqerr=%ld Id=%ld Idref=%ld | Vq=%ld Vd=%ld Vs=%ld(Q10) Vlim=%d sat:Iq=%d Id=%d Vs=%d | Udc=%.1fV mod=%d%% | BEMF_ff=%d Vd_ff=%.2fV Vq_ff=%.2fV psi=%.4f Lq=%.2fmH\r\n",
               (unsigned)c->IncPID_QAxis.P, (unsigned)c->IncPID_QAxis.I,
               (unsigned)c->IncPID_QAxis.PID_Div,
               (long)c->I_q_ref_filterd, (long)c->I_q,
               (long)(c->I_q_ref_filterd - c->I_q),
               (long)c->I_d, (long)c->I_d_ref,
               (long)c->V_q, (long)c->V_d, (long)vs_q10,
               (int)g_vs_limit,
               sat_iq, sat_id, sat_vs,
               udc_v, mod_pct,
               USE_BEMF_FF, vd_bemf_v, vq_bemf_v,
               c->ident_test.flux_psi, c->ident_test.Lq * 1000.0f);

        /* 行4: 三相电流反馈 (Q10, raw ADC, 校准 offset) + αβ 分量 */
        printf("[L200/4] Ia=%ld Ib=%ld Ic=%ld(Q10) | Ialpha=%ld Ibeta=%ld | raw a=%lu b=%lu | offA=%u offB=%u | theta_e=%ld order=%u\r\n",
               (long)c->I_a, (long)c->I_b, (long)c->I_c,
               (long)c->I_alpha, (long)c->I_beta,
               (unsigned long)c->Ia_raw, (unsigned long)c->Ib_raw,
               (unsigned)c->FlashData.Ia_offset, (unsigned)c->FlashData.Ib_offset,
               (long)c->theta_elec, (unsigned)c->FlashData.PhaseOrder);

        /* 行 4b: 弱磁状态 (USE_WEAK_MAGN=1 时有效, 关闭时全 0)
         *   id_weak  = Id 弱磁指令 (Q10, ≤0)
         *   Us_filt  = √(Vd²+Vq²) 一阶低通后 (Q10), 触发线 = 95% × Vlim
         *   vs_excess = Us_filt - 触发线 (>0 撞顶, <0 有余量)
         *   triggered = 1 表示当前在弱磁工作区 */
        int wmag_active = (c->compensation_weak < 0) ? 1 : 0;
        printf("[L200/4b] WMAG=%d id_weak=%ld(Q10) Us_filt=%lu(Q10) vs_excess=%ld trig=%d\r\n",
               USE_WEAK_MAGN,
               (long)c->compensation_weak,
               (unsigned long)c->Us,
               (long)c->voltage_error,
               wmag_active);

        /* 行5: PWM CCR + 双环斜坡 (区分饱和原因)
         *   ccr 接近 PWM_T 或 0 => SVPWM 撞调制极限
         *   spd_ramp != velocity_ref => SpeedLoopSmooth 在限速 (MIN_ACC_TIME 在拖)
         *   cur_ramp != I_q_ref     => CurrentLoopSmooth 在限速 (CURRENT_LOOP_MIN_ACC_TIME 在拖)
         */
        int dc2 = (PWM_T > 0) ? (int)((int64_t)c->CCR2 * 1000 / PWM_T) : 0;  /* 0.1% */
        int dc3 = (PWM_T > 0) ? (int)((int64_t)c->CCR3 * 1000 / PWM_T) : 0;
        int dc4 = (PWM_T > 0) ? (int)((int64_t)c->CCR4 * 1000 / PWM_T) : 0;
        printf("[L200/5] CCR=%lu/%lu/%lu (%d.%d/%d.%d/%d.%d%%) PWM_T=%d | SpdRamp now=%ld step=%ld vref=%ld | CurRamp now=%ld step=%ld iqref=%ld\r\n",
               (unsigned long)c->CCR2, (unsigned long)c->CCR3, (unsigned long)c->CCR4,
               dc2/10, dc2%10, dc3/10, dc3%10, dc4/10, dc4%10, PWM_T,
               (long)c->SpeedSmooth.NowVelocityRef, (long)c->SpeedSmooth.MaxVelAccEveryPrd,
               (long)c->velocity_ref,
               (long)c->CurrentSmooth.NowCurrentRef, (long)c->CurrentSmooth.MaxCurAccEveryPrd,
               (long)c->I_q_ref);

        /* 行6: 外层限幅 + 输出端反馈 (位置模式必看)
         *   MaxSpeed 太小会把 velocity_ref 夹下来 -> Iq_ref 永远到不了顶
         *   位置环饱和 (PosOut == PosLimit) 时扭矩堵在 INC_PID_POSITION_LIMIT
         */
        int32_t pos_out      = c->IncPID_Position.OutPut;
        int32_t pos_out_max  = c->IncPID_Position.OutputMax;
        int sat_pos          = (pos_out >=  pos_out_max - 1) ? 1
                             : (pos_out <= -pos_out_max + 1) ? -1 : 0;
        printf("[L200/6] MaxSpd=%ld(load Q10) OverI=%u(Q10) OverUdc=%u(0.1V) | PosRef=%ld PosOut=%ld dPosOut_eq=%ld(Q10rpm) | PosPID out=%ld/%ld sat=%d\r\n",
               (long)c->FlashData.MaxSpeed,
               (unsigned)Threshold.OverCurrent, (unsigned)Threshold.OverUdc,
               (long)c->position_ref, (long)c->real_position_out, (long)(c->dtheta_mech / 25),
               (long)pos_out, (long)pos_out_max, sat_pos);
        break;
    }
    default:
        break;
    }
}
