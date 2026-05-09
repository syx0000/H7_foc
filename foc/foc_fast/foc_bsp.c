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
#include "flash_port.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

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
            controller_eyou.I_q_ref = Data;
            controller_eyou.velocity_ref = 0;
        } else if (controller_eyou.controller_mode == PROFILE_VELOCITY_MOCE ||
                   controller_eyou.controller_mode == CYCLIC_SYNC_VELOCITY_MODE) {
            controller_eyou.velocity_ref = Data * 1024 * 101;
        } else if (controller_eyou.controller_mode == PROFILE_POSITION_MODE ||
                   controller_eyou.controller_mode == CYCLIC_SYNC_TORQUE_MODE) {
            controller_eyou.position_ref = Data * 1024;
        }
        printf("run mod_Target: %d, %d\r\n", controller_eyou.controller_mode, Data);
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
        printf("  elec0           %-16u %u\r\n", ram->elec_offest_0, fls->elec_offest_0);
        printf("  elec1           %-16u %u\r\n", ram->elec_offest_1, fls->elec_offest_1);
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
        printf("  InvDir          %-16d %d\r\n", ram->InvertDirflag, fls->InvertDirflag);
        printf("  BrakeT          %-16u %u\r\n", ram->brake_time, fls->brake_time);
        printf("  Crc             0x%08lX       0x%08lX\r\n", (unsigned long)ram->Crc, (unsigned long)fls->Crc);
        printf("[Size] sizeof(FlashSavedData)=%u\r\n", (unsigned)sizeof(FlashSavedData));
        printf("===== End =====\r\n");
        dbgLogFlag = 0;
        break;
    }
    default:
        break;
    }
}
