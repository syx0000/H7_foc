/**
 * @file    foc_api.c
 * @brief   模块功能描述
 * @author  yxsui
 * @date    2025-07-31
 * @version 1.0
 */

#include "foc_api.h"
// #include "cia402appl.h"  /* File not present in STM32 port */
#include "foc_data.h"
#include "foc_current_loop.h"
#include "foc_speed_loop.h"
#include "foc_position_loop.h"
#include "func_errMes.h"
#include "func_subprogram.h"
#include "ifly_fault_api.h"
#include "ifly_flux_ident.h"
#include "ifly_inertia_ident.h"
#include "encoder_calc.h"
#include "adc.h"
#include "tim.h"
#include <math.h>
#include <stdio.h>

extern Portection_Value Threshold;
extern Portection_Value Threshold_buffer;
extern ifly_Err_Pro_Type motorProValue;
// extern TCiA402Axis LocalAxes[MAX_AXES];  /* EtherCAT removed */
extern ErrMessgeStruct ErrMessge[ERRMESSGECOUNT];
extern FlashSavedData FlashData;

ControllerStruct controller_eyou = {0};

int8_t set_operation_mode(int8_t Mode) {
    return 0;
}

void motorRunStateUpdate(void) {
}

int8_t get_operation_mode(void) {
    return 0;
}

int32_t get_actual_position(void) {
    return 0;
}

int32_t get_actual_velocity(void) {
    return 0;
}

uint8_t set_velocity_ref(int32_t VelRef) {
    if (controller_eyou.ServoState.Bit.PdoRefreshing &&
        (controller_eyou.controller_mode == PROFILE_VELOCITY_MOCE ||
         controller_eyou.controller_mode == CYCLIC_SYNC_VELOCITY_MODE)) {
        int32_t max = (int32_t)controller_eyou.FlashData.MaxSpeed;
        if (VelRef >  max) VelRef =  max;
        if (VelRef < -max) VelRef = -max;
        controller_eyou.velocity_ref = VelRef;
    }
    return 1;
}

uint8_t set_velocity_ref_loop(int32_t VelRef) {
    int32_t max = (int32_t)controller_eyou.FlashData.MaxSpeed;
    if (VelRef >  max) VelRef =  max;
    if (VelRef < -max) VelRef = -max;
    controller_eyou.velocity_ref = VelRef;
    return 1;
}

int32_t get_velocity_ref(void) {
    return 0;
}

int32_t get_velocity_lim(void) {
    return 0;
}

int32_t set_torque_ref(int32_t TorRef) {
    if (controller_eyou.ServoState.Bit.PdoRefreshing &&
        (controller_eyou.controller_mode == PROFILE_TORQUE_MODE ||
         controller_eyou.controller_mode == CYCLIC_SYNC_TORQUE_MODE)) {
        int32_t max = (int32_t)controller_eyou.FlashData.MaxCurrent;
        if (TorRef >  max) TorRef =  max;
        if (TorRef < -max) TorRef = -max;
        controller_eyou.I_q_ref = TorRef;
    }
    return 1;
}

int32_t set_torque_ref_loop(int32_t TorRef) {
    int32_t max = (int32_t)controller_eyou.FlashData.MaxCurrent;
    if (TorRef >  max) TorRef =  max;
    if (TorRef < -max) TorRef = -max;
    controller_eyou.I_q_ref = TorRef;
    return 1;
}

int16_t get_max_current(void) {
    return 0;
}

uint32_t get_max_speed(void) {
    return 0;
}

int16_t get_actual_current(void) {
    return 0;
}

int32_t get_current_ref(void) {
    return 0;
}

int32_t postion_limit_check(int32_t PostionRef) {
    return 0;
}

int32_t set_postion_ref(int32_t PostionRef) {
    return 0;
}

uint32_t get_position_ref(void) {
    return 0;
}

uint32_t getControlErrFlag(void) {
    return 0;
}

uint32_t getControlStatusFlag(void) {
    return 0;
}

uint8_t motorPhaseCurrentStatesGet(void) {
    return 0;
}

uint8_t motorZeroPointStaGet(void) {
    return 0;
}

uint8_t motorSpeedOffsetGet(void) {
    return 0;
}

uint8_t motorSpeedOverGet(void) {
    return 0;
}

uint8_t motorPosOffsetGet(void) {
    return 0;
}

uint8_t flashReadErrGet(void) {
    return 0;
}

uint8_t getMotorRunState(void) {
    return 0;
}

uint8_t setMotorPhaseUErr(void) {
    return 0;
}

uint8_t setMotorPhaseVErr(void) {
    return 0;
}

uint8_t setMotorPhaseWErr(void) {
    return 0;
}

uint8_t resetMotorPhaseUErr(void) {
    return 0;
}

uint8_t resetMotorPhaseVErr(void) {
    return 0;
}

uint8_t resetMotorPhaseWErr(void) {
    return 0;
}

uint8_t getMotorPhaseUErr(void) {
    return 0;
}

uint8_t getMotorPhaseVErr(void) {
    return 0;
}

uint8_t getMotorPhaseWErr(void) {
    return 0;
}

uint8_t zeroPointLostCheck(void) {
    return 0;
}

uint8_t setCommnicationErr(void) {
    return 0;
}

uint8_t motorErrReset(void) {
    return 0;
}

uint8_t communicationErrReset(void) {
    return 0;
}

void motorRun(void) {
}

void motorStop(void) {
}

void motorStopProgress(uint8_t mod) {
}

uint8_t ServoStateFlagGudge(ControllerStruct* controller) {
    return 0;
}

BOOL get_motor_arrived_flag(void) {
    return 0;
}

void Init_foc(ControllerStruct* controller) {
    // 1. 控制器基础初始化
    controller_init(controller);

    // 2. 滤波器初始化
    controller->Speed_Filter.FilterInit = moving_average_create_s32;
    controller->Speed_Filter.FilterInit(&controller->Speed_Filter, SPEED_FILTER_DEPTH);
    controller->Speed_Filter.FilterRun = moving_average_filter_s32;

    InitCurrentShowFilter(controller);
    InitSpeedShowFilter(&controller->SpeedShowFilter);
    InitPositionShowFilter(controller);

    // 3. 斜坡滤波器初始化
    CurrentLoopSmoothInit(&controller->CurrentSmooth);
    SpeedLoopSmoothInit(&controller->SpeedSmooth);
    InitPosSmoothFunc(&controller->SmoothPosRef);

    // 4. 标志位初始化
    controller->ServoErrFlag.All_Flag = 0;
    controller->ServoState.All_Flag = 0;
    controller->ServoState.Bit.PdoRefreshing = 1;

    // 5. 从FLASH初始化数据
    InitFlashData(controller);

    // 6. 设置默认控制模式
    controller->controller_mode = DEFAULT_RUN_MODE;
    ResetControlData(controller);

    // 注意：跳过参数辨识部分（Rs, Ld, Lq, flux, inertia）
    // 开环测试不需要这些参数
}

uint8_t MC_Loop_Schedule(ControllerStruct* controller) {
    controller->count_loop++;

    /* 辨识模式下只跑电流环，避免速度环/位置环干扰 */
    if (!controller->ident_test.enable) {
        if ((controller->count_loop % POSITION_CALCULATE_DIV) == 0) {
            uint32_t tp0 = DWT_GetCycles();
            foc_position_close_loop(controller);
            uint32_t tp1 = DWT_GetCycles();
            g_adc_isr_t_pos = tp1 - tp0;
            if (g_adc_isr_t_pos > g_adc_isr_t_pos_max) g_adc_isr_t_pos_max = g_adc_isr_t_pos;
        }

        if ((controller->count_loop % VELOCETY_CALCULATE_DIV) == 0) {
            uint32_t tv0 = DWT_GetCycles();
            foc_velocity_close_loop(controller);
            uint32_t tv1 = DWT_GetCycles();
            g_adc_isr_t_vel = tv1 - tv0;
            if (g_adc_isr_t_vel > g_adc_isr_t_vel_max) g_adc_isr_t_vel_max = g_adc_isr_t_vel;
        }
    }

    if (controller->count_loop >= (POSITION_CALCULATE_DIV * VELOCETY_CALCULATE_DIV)) {
        controller->count_loop = 0;
    }

    uint32_t tc0 = DWT_GetCycles();
    foc_current_close_loop(controller);
    uint32_t tc1 = DWT_GetCycles();
    g_adc_isr_t_cur = tc1 - tc0;
    if (g_adc_isr_t_cur > g_adc_isr_t_cur_max) g_adc_isr_t_cur_max = g_adc_isr_t_cur;
    return 1;
}

void FocOpenTest(ControllerStruct* controller,
                 uint8_t ModelChoose,
                 int16_t v_d,
                 int16_t v_q,
                 uint16_t IaSampleValue,
                 uint16_t IbSampleValue) {
    static int32_t theta_openloop = 0;
    int32_t theta_used;
    (void)IaSampleValue;
    (void)IbSampleValue;

    // 电角度生成
    if (ModelChoose == 0) {
        theta_openloop += 40;
        if (theta_openloop >= 65536) theta_openloop -= 65536;
        theta_used = theta_openloop;
    } else {
        theta_used = controller->theta_elec;
    }

    // 设置相电压（SVPWM）
    set_phase_voltage(controller, v_d, v_q, theta_used);
}

#define RS_IDENT_VOLTAGE        1.0f
#define RS_IDENT_SETTLE_MS      200
#define RS_IDENT_DURATION_MS    1000
#define RS_IDENT_SAMPLE_PRD_MS  100
#define RS_IDENT_TOL_OHM        0.001f

#define DAXIS_ALIGN_VOLTAGE 1.0f
#define DAXIS_ALIGN_MS      1000

void alignDAxis(ControllerStruct* controller) {
    /* 进辨识通道：ident_test.enable=1 让 ISR 旁路 PI 直通 V_d/V_q，
       同时 encoder_calc 不再覆盖 theta_elec。
       foc_run=1 让 ISR 走 MC_Loop_Schedule → foc_current_close_loop → ident 分支，
       避免 FocOpenTest 的旋转 theta 把 CCR 刷掉。 */
    uint8_t old_foc_run = controller->foc_run;

    controller->ident_test.enable = 1;
    controller->ident_test.amplitude = 0;                 /* DC 模式 */
    controller->ident_test.settle_samples = 0xFFFFFFFF;   /* 防止 ISR 自动把 enable 清零 */
    controller->ident_test.measure_samples = 0;
    controller->ident_test.sample_count = 0;

    controller->theta_elec = 0;
    controller->V_d = (int32_t)(DAXIS_ALIGN_VOLTAGE * 1024);
    controller->V_q = 0;
    controller->foc_run = 1;

    HAL_Delay(DAXIS_ALIGN_MS);

    controller->V_d = 0;
    controller->V_q = 0;
    set_phase_voltage(controller, 0, 0, 0);
    HAL_Delay(50);

    controller->ident_test.enable = 0;
    controller->foc_run = old_foc_run;
    printf("D-axis aligned (V=%.1fV, %dms)\r\n", DAXIS_ALIGN_VOLTAGE, DAXIS_ALIGN_MS);
}

float measurePhaseResistance(ControllerStruct* controller) {
    uint8_t old_foc_run = controller->foc_run;

    /* 进辨识通道：enable=1 旁路 PI + 编码器不覆盖 theta_elec。
       settle_samples 设极大值，防止 ISR 端 sample_count 自增后自动把 enable 清掉。 */
    controller->ident_test.enable = 1;
    controller->ident_test.done   = 0;
    controller->ident_test.amplitude = 0;
    controller->ident_test.settle_samples  = 0xFFFFFFFF;
    controller->ident_test.measure_samples = 0;
    controller->ident_test.sample_count    = 0;
    controller->ident_test.i_sin = 0;
    controller->ident_test.i_cos = 0;

    /* 钉死电角度 → 电压矢量固定在 A 相方向；theta=0 时 Park 变换下 I_d 应等价 I_a */
    controller->theta_elec = 0;
    controller->V_d = (int32_t)(RS_IDENT_VOLTAGE * 1024);
    controller->V_q = 0;
    controller->foc_run = 1;

    /* 等电流稳态 */
    HAL_Delay(RS_IDENT_SETTLE_MS);

    /* 5s 采样窗口，每 100ms 取一次，共 50 点；同时累加 I_a 和 I_d */
    const int N = RS_IDENT_DURATION_MS / RS_IDENT_SAMPLE_PRD_MS;
    int64_t sum_ia = 0;
    int64_t sum_id = 0;
    for (int i = 0; i < N; i++) {
        HAL_Delay(RS_IDENT_SAMPLE_PRD_MS);
        sum_ia += controller->I_a;
        sum_id += controller->I_d;
    }

    controller->ident_test.enable = 0;
    controller->V_d = 0;
    controller->V_q = 0;
    set_phase_voltage(controller, 0, 0, 0);
    controller->foc_run = old_foc_run;

    float Ia_avg = (float)sum_ia / N / 1024.0f;
    float Id_avg = (float)sum_id / N / 1024.0f;
    float R_ia = (fabsf(Ia_avg) > 0.01f) ? (RS_IDENT_VOLTAGE / fabsf(Ia_avg)) : 0.0f;
    float R_id = (fabsf(Id_avg) > 0.01f) ? (RS_IDENT_VOLTAGE / fabsf(Id_avg)) : 0.0f;
    float diff = fabsf(R_ia - R_id);
    float Rs = (R_ia + R_id) * 0.5f;

    controller->ident_test.Rs = Rs;
    printf("Rs: R(via Ia)=%.4f  R(via Id)=%.4f  diff=%.4f  %s  (Ia=%.3fA Id=%.3fA)\r\n",
           R_ia, R_id, diff,
           (diff < RS_IDENT_TOL_OHM) ? "PASS" : "FAIL",
           Ia_avg, Id_avg);
    return Rs;
}


// 高频注入配置 (用于电感辨识)
#define INJ_FREQ_HZ 600.0f          // 注入频率（Hz），采样10kHz时每周期20点，同步检测相位干净
#define INJ_VOLTAGE_AMPL 0.5f       // 注入电压幅值（V）

void measurePhaseInductanceAC(ControllerStruct* controller, float Rs) {
    uint8_t old_foc_run = controller->foc_run;

    /* 钉死电角度 → AC 注入方向固定在 A 相 / B 相方向，避免编码器未标定时方向随机 */
    controller->theta_elec = 0;

    /* 先 init（把 ident.enable 置 1，填好参数），再开 foc_run，
       避免 ISR 先看到 foc_run=1、enable=0 而走一次 PI 分支 */
    ident_inductance_init(&controller->ident_test, 0, INJ_FREQ_HZ,
                          INJ_VOLTAGE_AMPL * 1024, Rs);
    controller->foc_run = 1;
    while (!controller->ident_test.done) {
        HAL_Delay(1);
    }
    ident_inductance_compute(&controller->ident_test);
    float Ld = controller->ident_test.Ld;
    double v_amp_d = sqrt((double)controller->ident_test.v_sin * controller->ident_test.v_sin +
                          (double)controller->ident_test.v_cos * controller->ident_test.v_cos);
    double i_amp_d = sqrt((double)controller->ident_test.i_sin * controller->ident_test.i_sin +
                          (double)controller->ident_test.i_cos * controller->ident_test.i_cos);
    float Z_d = (i_amp_d > 0) ? (float)(v_amp_d / i_amp_d) : 0.0f;

    /* 再 pin 一次 theta，避免 d 轴测完到 q 轴 init 之间编码器覆盖 */
    controller->theta_elec = 0;

    // q轴电感
    ident_inductance_init(&controller->ident_test, 1, INJ_FREQ_HZ,
                          INJ_VOLTAGE_AMPL * 1024, Rs);
    while (!controller->ident_test.done) {
        HAL_Delay(1);
    }
    ident_inductance_compute(&controller->ident_test);
    float Lq = controller->ident_test.Lq;
    double v_amp_q = sqrt((double)controller->ident_test.v_sin * controller->ident_test.v_sin +
                          (double)controller->ident_test.v_cos * controller->ident_test.v_cos);
    double i_amp_q = sqrt((double)controller->ident_test.i_sin * controller->ident_test.i_sin +
                          (double)controller->ident_test.i_cos * controller->ident_test.i_cos);
    float Z_q = (i_amp_q > 0) ? (float)(v_amp_q / i_amp_q) : 0.0f;

    /* ISR 到达 settle+measure 会自动把 enable 清零，这里再显式清一次做保底 */
    controller->ident_test.enable = 0;
    set_phase_voltage(controller, 0, 0, 0);
    controller->foc_run = old_foc_run;

    printf("Ld = %.4f mH  Z_d=%.4fOhm  (freq=%.0fHz Rs=%.4fOhm)\r\n",
           Ld * 1000.0f, Z_d, INJ_FREQ_HZ, Rs);
    printf("Lq = %.4f mH  Z_q=%.4fOhm  (freq=%.0fHz Rs=%.4fOhm)\r\n",
           Lq * 1000.0f, Z_q, INJ_FREQ_HZ, Rs);

    controller->ident_test.Ld = Ld;
    controller->ident_test.Lq = Lq;
}

/* Flash 缓存的电机参数辨识: Flash Flag 有效则跳过, 无效则辨识并写回。
 * 用 union 把 float 装进 Flash 的 int32_t 预留字段:
 *   temp1=Rs, temp2=Ld, temp3=Lq, temp7=psi_f(磁链), temp8=J(惯量)
 * MotorParamFlag == OFFEST_IS_CORRECTED_FLAG 表示 Rs/Ld/Lq 已落盘。 */
void identifyMotorParamsCached(ControllerStruct* controller) {
    union { float f; int32_t i; } u_rs, u_ld, u_lq, u_psi, u_j;

    if (controller->FlashData.MotorParamFlag == OFFEST_IS_CORRECTED_FLAG) {
        u_rs.i  = controller->FlashData.temp1;
        u_ld.i  = controller->FlashData.temp2;
        u_lq.i  = controller->FlashData.temp3;
        controller->ident_test.Rs = u_rs.f;
        controller->ident_test.Ld = u_ld.f;
        controller->ident_test.Lq = u_lq.f;
        printf("Motor params loaded from Flash: Rs=%.4f Ohm  Ld=%.4f mH  Lq=%.4f mH\r\n",
               u_rs.f, u_ld.f * 1000.0f, u_lq.f * 1000.0f);

        if (controller->FlashData.FluxIdentFlag == OFFEST_IS_CORRECTED_FLAG) {
            u_psi.i = controller->FlashData.temp7;
            controller->ident_test.flux_psi = u_psi.f;
            printf("  psi_f=%.6f Wb\r\n", u_psi.f);
        }
        if (controller->FlashData.InertiaIdentFlag == OFFEST_IS_CORRECTED_FLAG) {
            u_j.i = controller->FlashData.temp8;
            printf("  J=%.3e kg*m^2\r\n", u_j.f);
        }
        return;
    }

    printf("MotorParamFlag invalid (0x%04X), running identification...\r\n",
           (unsigned)controller->FlashData.MotorParamFlag);
    float Rs = measurePhaseResistance(controller);
    measurePhaseInductanceAC(controller, Rs);

    u_rs.f = controller->ident_test.Rs;
    u_ld.f = controller->ident_test.Ld;
    u_lq.f = controller->ident_test.Lq;
    controller->FlashData.temp1          = u_rs.i;
    controller->FlashData.temp2          = u_ld.i;
    controller->FlashData.temp3          = u_lq.i;
    controller->FlashData.MotorParamFlag = OFFEST_IS_CORRECTED_FLAG;
    WriteRunDataToFlash(controller, MOTORID0_RUN_DATA_ADDRESS);
    printf("Rs/Ld/Lq saved to Flash\r\n");
}

void autoTuneCurrentLoopPI(float Rs, float Ld, float Lq) {
    (void)Ld;
    float Ts = 1.0f / FOC_FREQUENCY;
    float omega_bw = 2.0f * M_PIf * CURRENT_LOOP_TARGET_BW_HZ;

    uint16_t Kp = (uint16_t)(omega_bw * Lq * DEFAULT_PID_DIV + 0.5f);
    uint16_t Ki = (uint16_t)(omega_bw * Rs * DEFAULT_PID_DIV * Ts + 0.5f);

    if (Kp < 10) Kp = 10;
    if (Kp > 500) Kp = 500;
    if (Ki < 1) Ki = 1;
    if (Ki > 200) Ki = 200;

    // set_current_loop_kp_ec(Kp);
    // set_current_loop_ki_ec(Ki);

    printf("AutoTune Current: BW=%dHz, Lq=%.3fmH, Rs=%.3fOhm -> Kp=%d, Ki=%d\r\n",
           CURRENT_LOOP_TARGET_BW_HZ, Lq * 1000.0f, Rs, Kp, Ki);
}

#define SPEED_LOOP_TARGET_BW_HZ    60
#define SPEED_LOOP_ZERO_RATIO      8
/* IMC 模型假设速度环被控对象为纯积分器 J/s, 电流环视为单位增益。
 * 实测真实系统在目标带宽附近存在额外相位滞后:
 *   - 电流环 1710Hz 在 60Hz 处贡献 ~2 度滞后
 *   - 编码器速度滤波 (滑动均值 16 拍 @ 5kHz) 贡献 ~12 度滞后
 *   - 减速箱反向间隙 + 机械刚度有限, 50Hz 附近可能有结构共振
 * 直接用 IMC Kp (Kp=2600 实测) 会让谐振峰 +6dB, 需要按经验系数缩减。
 * 0.6 系数由实测拟合: Kp=1500 Ki=10 对应峰值 1.8dB / 带宽 45Hz, 是合格区域。 */
#define SPEED_LOOP_SAFETY_FACTOR   0.6f

void autoTuneSpeedLoopPI(float J, float psi_f, uint8_t pp) {
    if (pp == 0 || psi_f <= 0.0f || J <= 0.0f) {
        printf("AutoTune Speed: invalid inputs (J=%.3e psi_f=%.6f Pp=%d)\r\n", J, psi_f, pp);
        return;
    }

    float fs_w     = (float)SPEED_LOOP_FRE;
    float Ts_w     = 1.0f / fs_w;
    float Kt       = 1.5f * (float)pp * psi_f;
    float omega_c  = 2.0f * M_PIf * (float)SPEED_LOOP_TARGET_BW_HZ;
    float omega_i  = omega_c / (float)SPEED_LOOP_ZERO_RATIO;

    /* IMC 极零点对消 + 安全系数: 补偿实测系统多出的相位滞后 */
    float Kp_w     = J * omega_c / Kt * SPEED_LOOP_SAFETY_FACTOR;
    float Ki_w     = Kp_w * omega_i;

    const float scale = 2.0f * M_PIf / 60.0f;
    float P_f      = Kp_w * scale * (float)DEFAULT_PID_SPEED_DIV;
    float I_f      = Ki_w * scale * (float)DEFAULT_PID_SPEED_DIV * Ts_w;

    uint16_t Kp = (uint16_t)(P_f + 0.5f);
    uint16_t Ki = (uint16_t)(I_f + 0.5f);

    if (Kp < 1)     Kp = 1;
    if (Kp > 30000) Kp = 30000;
    if (Ki < 1)     Ki = 1;
    if (Ki > 1000)  Ki = 1000;

    // set_speed_loop_kp_ec((int32_t)Kp);
    // set_speed_loop_ki_ec((int32_t)Ki);

    printf("AutoTune Speed: BW=%dHz J=%.3e psif=%.6f Pp=%d -> Kp=%d Ki=%d (not applied)\r\n",
           SPEED_LOOP_TARGET_BW_HZ, J, psi_f, pp, Kp, Ki);
}

/* 位置环 PI 自整定
 * 经验公式 (无被控对象模型, 完全基于带宽和采样周期):
 *   位置环目标带宽 ω_c = 2π × POSITION_LOOP_TARGET_BW_HZ
 *   Kp_pos = ω_c × PID_Div × SAFETY_FACTOR
 *   Ki_pos = Kp_pos × (ω_c / ZERO_RATIO) × Ts_pos
 *
 * 设计经验:
 *   - 位置环带宽 ≈ 速度环带宽 / 4 (避免内环未跟踪导致位置积分饱和)
 *     当前速度环实测 BW ≈ 45Hz → 位置环目标 12Hz 较稳
 *   - 位置环没有 PHU/motor_h7 的 autoTune 参考, 因为对机械非线性敏感:
 *     减速箱反向间隙 + 负载摩擦 + 编码器分辨率限制
 *   - SAFETY_FACTOR=0.4: 实测最佳值, 给出 Kp=3016 Ki=9, 带宽 25.6Hz, 谐振峰 0.58dB。
 *     Kp > ~3500 后位置环输出 velocity_ref 过大, 速度环 I_q_ref 撞 MaxCurrent 限幅,
 *     系统进入极限环振荡 (0.6/0.8 系数实测均发散)。带宽受机械系统硬限, 非控制器激进度。
 *   - 零点比 10 比速度环 8 更保守, 避免低频隆起 */
#define POSITION_LOOP_TARGET_BW_HZ   12
#define POSITION_LOOP_ZERO_RATIO     10
#define POSITION_LOOP_SAFETY_FACTOR  0.4f

void autoTunePositionLoopPI(void) {
    float fs_p    = (float)(FOC_FREQUENCY / POSITION_CALCULATE_DIV);  /* 2500Hz */
    float Ts_p    = 1.0f / fs_p;
    float omega_c = 2.0f * M_PIf * (float)POSITION_LOOP_TARGET_BW_HZ;
    float omega_i = omega_c / (float)POSITION_LOOP_ZERO_RATIO;

    float Kp_p    = omega_c * (float)DEFAULT_PID_POSITION_DIV * POSITION_LOOP_SAFETY_FACTOR;
    float Ki_p    = Kp_p * omega_i * Ts_p;

    uint32_t Kp = (uint32_t)(Kp_p + 0.5f);
    uint32_t Ki = (uint32_t)(Ki_p + 0.5f);

    if (Kp < 100)   Kp = 100;
    if (Kp > 60000) Kp = 60000;
    if (Ki < 1)     Ki = 1;
    if (Ki > 10000) Ki = 10000;

    // /* 同步写入 IncPID 运行结构和 FlashData (掉电保存) */
    // controller_eyou.IncPID_Position.P = Kp;
    // controller_eyou.IncPID_Position.I = Ki;
    // controller_eyou.FlashData.Position_Kp = Kp;
    // controller_eyou.FlashData.Position_Ki = Ki;

    printf("AutoTune Position: BW=%dHz fs=%.0fHz Div=%d -> Kp=%u Ki=%u\r\n",
           POSITION_LOOP_TARGET_BW_HZ, fs_p, DEFAULT_PID_POSITION_DIV,
           (unsigned)Kp, (unsigned)Ki);
    printf("  Hint: run pos bwtest to verify, tune manually if peak > 3dB\r\n");
}

uint32_t Set_Position_Limit(uint32_t Limit) {
    return 0;
}

uint16_t get_board_temp(void) {
    return 0;
}

uint16_t Pid_Control_Enable(uint16_t pid_key) {
    return 0;
}

int32_t Get_error_speed(void) {
    return 0;
}

int32_t set_error_eeprom(void) {
    return 0;
}

int16_t Quick_stop_option(int16_t code) {
    return 0;
}

int16_t Shutdown_code(int16_t key) {
    return 0;
}

int16_t Disable_Operation(int16_t stop) {
    return 0;
}

int16_t Halt_Option(int16_t option) {
    return 0;
}

int16_t Fault_React(int16_t react) {
    return 0;
}

int16_t Sensor_select(int16_t select) {
    return 0;
}

uint16_t Speed_max_warn(void) {
    return 0;
}

int32_t Home_Offset(void) {
    return 0;
}

uint32_t Set_QuickStop_deceleration(uint32_t deceleration) {
    return 0;
}

uint32_t Set_Torque_slope(uint32_t slope) {
    return 0;
}

uint8_t Set_Homing_Method(uint8_t Method) {
    return 0;
}

uint32_t Set_Homing_Speeds(uint32_t speed) {
    return 0;
}

uint32_t Set_Homing_Speeds_zero(uint32_t speed) {
    return 0;
}

uint32_t Set_Homing_Acceleration(uint32_t Acceleration) {
    return 0;
}

uint32_t Set_Torque_Offset(uint32_t offset) {
    return 0;
}

uint32_t Set_Maximum_acceleration(uint32_t accele) {
    return 0;
}

uint32_t Set_Maximum_deceleration(uint32_t decele) {
    return 0;
}

uint16_t Set_Positive_torque_limit(uint16_t limit) {
    return 0;
}

uint32_t Set_Negative_torque_limit(uint32_t limit) {
    return 0;
}

uint16_t Set_Position_Option_Code(uint16_t code) {
    return 0;
}

uint16_t Get_Follow_offset(void) {
    return 0;
}

int32_t Get_Pos_Demand(void) {
    return 0;
}

uint16_t Set_Break_Dealy_Time(uint16_t BreakTime) {
    return 0;
}

uint32_t Get_Motor_Encoder_Value(void) {
    return 0;
}

uint32_t Get_Double_Encoder_Value(void) {
    return 0;
}

uint32_t Reset_objReset_Output_Encoder(uint32_t reset) {
    static uint32_t last_reset_value = 0;

    if (reset == 1 && last_reset_value == 0) {
        if (controller_eyou.controller_mode == HOMING_MODE) {
            controller_eyou.FlashData.mech_offest_out = controller_eyou.old_angle_count_out_raw;
            controller_eyou.circle_count_out = 0;

            Encoder_out_data_Reset(controller_eyou.FlashData.MaxPositionLimit,
                                   controller_eyou.FlashData.MinPositionLimit);
        }
    }

    last_reset_value = reset;
    return 0;
}

int16_t Motion_profile_type(int16_t type) {
    return 0;
}

int16_t Interpolatedsubmodeselect(int16_t mode) {
    return 0;
}

int8_t InterpolationIndex(int8_t Index) {
    return 0;
}

uint8_t Inter_polation_Period(uint8_t Period) {
    return 0;
}

uint32_t Get_Supported_Drive_Modes(void) {
    return 0;
}

uint16_t set_STO_controll(uint16_t STO) {
    return 0;
}

uint16_t get_STO_status(void) {
    return 0;
}

uint16_t set_system_restart(uint16_t reset) {
    return 0;
}

uint16_t get_Historical_Fault(uint16_t infor) {
    return 0;
}

uint8_t reset_default_value(uint8_t value) {
    return 0;
}

void FlashData_recover_default(ControllerStruct* controller) {
}

int32_t set_Maxposition_Limit(int32_t Limit) {
    return 0;
}

int32_t set_Minposition_Limit(int32_t Limit) {
    return 0;
}

int32_t set_speed_loop_kp_ec(int32_t kp) {
    if (kp < 0) return -1;
    controller_eyou.IncPID_Speed.P = kp;
    controller_eyou.FlashData.Speed_Kp = kp;
    return kp;
}

int32_t set_speed_loop_ki_ec(int32_t ki) {
    if (ki < 0) return -1;
    controller_eyou.IncPID_Speed.I = ki;
    controller_eyou.FlashData.Speed_Ki = ki;
    return ki;
}

int32_t set_speed_loop_kd_ec(int32_t kd) {
    if (kd < 0) return -1;
    controller_eyou.IncPID_Speed.D = kd;
    controller_eyou.FlashData.Speed_Kd = kd;
    return kd;
}

int32_t set_pos_error_ff_gain(int32_t gain) {
    controller_eyou.pos_err_ff_gain = gain;
    controller_eyou.FlashData.PosErrFF_Kp = gain;
    return gain;
}

int32_t set_position_loop_kp_ec(int32_t kp) {
    if (kp < 0) return -1;
    controller_eyou.IncPID_Position.P = kp;
    controller_eyou.FlashData.Position_Kp = kp;
    return kp;
}

int32_t set_position_loop_ki_ec(int32_t ki) {
    if (ki < 0) return -1;
    controller_eyou.IncPID_Position.I = ki;
    controller_eyou.FlashData.Position_Ki = ki;
    return ki;
}

int32_t set_position_loop_kd_ec(int32_t kd) {
    if (kd < 0) return -1;
    controller_eyou.IncPID_Position.D = kd;
    controller_eyou.FlashData.Position_Kd = kd;
    return kd;
}

int32_t set_current_loop_kp_ec(int32_t kp) {
    if (kp < 0) return -1;
    controller_eyou.IncPID_QAxis.P = kp;
    controller_eyou.IncPID_DAxis.P = kp;
    controller_eyou.FlashData.Current_Kp = kp;
    return kp;
}

int32_t set_current_loop_ki_ec(int32_t ki) {
    if (ki < 0) return -1;
    controller_eyou.IncPID_QAxis.I = ki;
    controller_eyou.IncPID_DAxis.I = ki;
    controller_eyou.FlashData.Current_Ki = ki;
    return ki;
}

int32_t set_current_loop_kd_ec(int32_t kd) {
    if (kd < 0) return -1;
    controller_eyou.IncPID_QAxis.D = kd;
    controller_eyou.IncPID_DAxis.D = kd;
    controller_eyou.FlashData.Current_Kd = kd;
    return kd;
}
