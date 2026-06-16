/**
 * @file    can_wly.c
 * @brief   万里扬FDCAN通信协议V1.7 - 从站实现
 */
#include "can_protocol_sel.h"
#include "can_wly.h"
#include "can_debug.h"
#include "fdcan.h"
#include "foc_api.h"
#include "foc_controller.h"
#include "foc_data.h"
#include "foc_bsp.h"
#include "ifly_fault.h"
#include "ifly_fault_api.h"
#include <string.h>
#include <math.h>
#include <stdio.h>

/* ========== 内部状态 ========== */
static uint8_t s_node_id = CAN_WLY_ID_DEFAULT;

can_wly_limits_t g_can_wly_lim = {
    .pos_min = -7.0f,  .pos_max = 7.0f,
    .spd_min = -20.0f, .spd_max = 20.0f,
    .tq_min  = -500.0f, .tq_max = 500.0f,
    .kp_min  = 0.0f,   .kp_max  = 500.0f,
    .kd_min  = 0.0f,   .kd_max  = 20.0f,
};

float g_can_wly_kt_out = 1.0f;

/* 主动上报模式 (0x2F05 写 2 开启). 1ms 周期上报 0x100+ID 状态帧 */
static uint8_t s_auto_report = 0;

/* 0x2F05 cmd=1 触发的电流带宽测试: 单频正弦注入 + 0x7FD 逐拍上报 */
static volatile uint8_t  s_test_active     = 0;          /* 1 = 注入 + 上报中 */
static uint32_t          s_test_freq_hz    = 100;        /* 0x2F06 默认 100 Hz */
static uint32_t          s_test_ampl_q10   = 256;        /* 0x2F07 默认 0.25 A (Q10) */
static volatile uint32_t s_test_phase_acc  = 0;
static uint32_t          s_test_phase_step = 0;          /* freq/fs * 2^32, 启动时算好 */
static uint32_t          s_test_tx_fail_cnt = 0;         /* 0x7FD FIFO 满丢帧计数 */
static uint32_t          s_test_tx_ok_cnt   = 0;         /* 0x7FD 发送成功计数 */

/* 发送失败计数器 (调试用) */
static uint32_t s_tx_fail_count = 0;

/* Iq 反馈滤波 (Q10, 一阶 IIR α=1/8, 10kHz → fc≈200Hz)
 * 仅用于对外上报 (CAN 0x100/0x7FE 状态帧). 控制环不能用,
 * 否则等于在闭环里塞滞后, 影响电流环带宽和稳定裕度 */
static volatile int32_t s_iq_fb_filt_q10 = 0;

/* 上位机最近一次下发的扭矩指令原始值 (N·m), 仅供 0x100 上报回显
 * 0x300 PROFILE_TORQUE_MODE / 0x500 MIT_PD_MODE 写入,
 * 用 s_last_torque_cmd_nm 以源指令为准, 不经过 LUT 反推 */
static volatile float s_last_torque_cmd_nm = 0.0f;

/* CAN 超时保护：收到有效帧时重置，1ms 递减，归零停机 */
#define CAN_TIMEOUT_MS 200
#define MIT_TIMEOUT_MS 20
static volatile uint16_t s_can_timeout_cnt = 0;
static volatile uint16_t s_mit_timeout_cnt = 0;
static uint8_t s_can_timeout_enabled = 0;
uint8_t g_can_timeout_force_disable = 1;  /* CAN 超时保护已启用 */

/* ========== 访问全局控制器 ========== */
extern ControllerStruct controller_eyou;
extern ifly_Err_Pro_Type motorProValue;
extern Portection_Value Threshold;

/* forward decl */
static void can_dbg_push(uint32_t id, const uint8_t *data, uint32_t len, uint8_t dir);

/* ========== 小工具: float <-> uint 定点 ========== */
static uint32_t float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    if (span <= 0.0f) return 0;
    if (x < x_min) x = x_min; else if (x > x_max) x = x_max;
    uint32_t mask = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    return (uint32_t)((x - x_min) * (float)mask / span);
}

static float uint_to_float(uint32_t v, float x_min, float x_max, uint8_t bits) {
    uint32_t mask = (bits >= 32) ? 0xFFFFFFFFu : ((1u << bits) - 1u);
    return (float)v / (float)mask * (x_max - x_min) + x_min;
}

/* float <-> 4 字节 LSB-first (协议中 Data[31:24] 在 D[7], Data[7:0] 在 D[4]) */
static float bytes_to_float_le(const uint8_t *p) {
    union { uint32_t u; float f; } c;
    c.u = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
          ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return c.f;
}
static void float_to_bytes_le(float v, uint8_t *p) {
    union { uint32_t u; float f; } c;
    c.f = v;
    p[0] = c.u & 0xFF;
    p[1] = (c.u >> 8) & 0xFF;
    p[2] = (c.u >> 16) & 0xFF;
    p[3] = (c.u >> 24) & 0xFF;
}

/* ========== 单位换算: STM32 内部 <-> 协议 SI ========== */
/* 内部: position 1°/1024, velocity 内部按 "电机端 rpm*1024*GR", current Q10 */
#define DEG_PER_1024     (1.0f / 1024.0f)
#define RPM_TO_RAD_S     (2.0f * (float)M_PI / 60.0f)
#define DEG_TO_RAD       ((float)M_PI / 180.0f)

/* 位置: 内部 (输出端 1°/1024) -> rad */
static float pos_int_to_rad(int32_t pos_out) {
    return (float)pos_out * DEG_PER_1024 * DEG_TO_RAD;
}
/* 位置: rad -> 内部 (输出端 1°/1024) */
static int32_t pos_rad_to_int(float rad) {
    return (int32_t)(rad / DEG_TO_RAD * 1024.0f);
}
/* 速度: dtheta_mech (电机端 rpm*1024*GR) -> rad/s 输出端 (反馈路径)
   不再使用 dtheta_mech_out, 改用电机端折算避免负载端编码器低频抖动 */
static float vel_motor_to_load_rad_s(int32_t dtheta_motor) {
    float rpm_load = (float)dtheta_motor / (1024.0f * CAN_WLY_GR);
    return rpm_load * RPM_TO_RAD_S;
}
/* 速度: 输出端 rad/s -> velocity_ref (rpm*1024*GR) (指令路径) */
static int32_t vel_rad_s_to_int(float rad_s) {
    float rpm_out = rad_s / RPM_TO_RAD_S;
    return (int32_t)(rpm_out * 1024.0f * CAN_WLY_GR);
}
/* ========== Kt 标定 LUT (电机端 Iq A <-> 输出端 Torque N·m) ==========
 * 单调递增, 仅存正半轴, 负值靠 sign 对称
 * 端点之外按相邻段斜率线性外推
 * 反查依赖 t_Nm 单调, 若饱和翻折需把表截到翻折点之前
 * g_can_wly_kt_out 作为整体缩放因子, 用于不同电机批次 ±x% 微调, 默认 1.0f
 * 数据来源: TODO 测功机/拉力计静态标定后替换 (当前是占位线性) */
typedef struct { float i_A; float t_Nm; } kt_pt_t;

static const kt_pt_t s_kt_lut[] = {
    {  0.0f,  0.00f},
    {  5.0f,  9.7f},
    { 10.0f,  21.7f},
    { 15.0f,  33.6f},
    { 20.0f,  45.2f},
    { 25.0f,  56.7f},
    { 30.0f,  67.8f},
    { 35.0f,  78.6f},
    { 40.0f,  88.9f},
    { 45.0f,  98.8f},
    { 53.0f,  101.7f},
    { 58.0f,  110.0f},
    { 79.0f,  150.0f},
    { 100.0f, 190.0f},
};
#define KT_LUT_N (sizeof(s_kt_lut) / sizeof(s_kt_lut[0]))

/* I (A, 带符号) -> T (N·m, 带符号) */
float can_wly_iA_to_Nm(float i_A) {
    float s = (i_A < 0.0f) ? -1.0f : 1.0f;
    float a = (i_A < 0.0f) ? -i_A : i_A;
    float t;
    if (a <= s_kt_lut[0].i_A) {
        t = s_kt_lut[0].t_Nm;
    } else {
        uint32_t i;
        for (i = 1; i < KT_LUT_N; i++) {
            if (a <= s_kt_lut[i].i_A) break;
        }
        if (i >= KT_LUT_N) i = KT_LUT_N - 1;
        float di = s_kt_lut[i].i_A - s_kt_lut[i - 1].i_A;
        float k  = (di > 0.0f) ? (s_kt_lut[i].t_Nm - s_kt_lut[i - 1].t_Nm) / di : 0.0f;
        t = s_kt_lut[i - 1].t_Nm + k * (a - s_kt_lut[i - 1].i_A);
    }
    return s * t * g_can_wly_kt_out;
}

/* T (N·m, 带符号) -> I (A, 带符号), LUT t 单调递增 */
float can_wly_Nm_to_iA(float t_Nm) {
    if (g_can_wly_kt_out == 0.0f) return 0.0f;
    float t_scaled = t_Nm / g_can_wly_kt_out;
    float s = (t_scaled < 0.0f) ? -1.0f : 1.0f;
    float t = (t_scaled < 0.0f) ? -t_scaled : t_scaled;
    float a;
    if (t <= s_kt_lut[0].t_Nm) {
        a = s_kt_lut[0].i_A;
    } else {
        uint32_t i;
        for (i = 1; i < KT_LUT_N; i++) {
            if (t <= s_kt_lut[i].t_Nm) break;
        }
        if (i >= KT_LUT_N) i = KT_LUT_N - 1;
        float dt = s_kt_lut[i].t_Nm - s_kt_lut[i - 1].t_Nm;
        float k  = (dt > 0.0f) ? (s_kt_lut[i].i_A - s_kt_lut[i - 1].i_A) / dt : 0.0f;
        a = s_kt_lut[i - 1].i_A + k * (t - s_kt_lut[i - 1].t_Nm);
    }
    return s * a;
}

/* 转矩: q轴电流(Q10) -> N·m */
static float tq_iq_to_nm(int32_t iq_q10) {
    return can_wly_iA_to_Nm((float)iq_q10 / 1024.0f);
}
/* 转矩: N·m -> q轴电流(Q10) */
static int32_t tq_nm_to_iq(float nm) {
    return (int32_t)(can_wly_Nm_to_iA(nm) * 1024.0f);
}

/* ========== 0x100+ID 状态帧 (32 字节, FDCAN 自动 padding 到 48B) ==========
 * D[0..2]   Pact 反馈 [23:0] (float->定点, 按 PosMin/Max)
 * D[3..4]   Vact 反馈 [15:0] (float->定点, 按 SpdMin/Max)
 * D[5..6]   Tact 反馈 [15:0] (float->定点, 按 TqMin/Max, 来自 Iq 滤波 + Kt LUT)
 * D[7..8]   Err1[15:0]
 * D[9]      Err2[7:0] (只占低字节, 高16位保留, 与表头一致)
 * D[10]     warn (Bit0=MOS过温, Bit1=电机过温)
 * D[11]     STA (Bit0=使能, Bit1=故障, Bit2=警告, Bit3=到达)
 * D[12..14] Pcmd 指令 [23:0] (position_ref → PosMin/Max 24bit)
 * D[15..16] Vcmd 指令 [15:0] (velocity_ref → SpdMin/Max 16bit)
 * D[17..18] Tcmd 指令 [15:0] (上位机最近一次 0x300/0x500 下发的 N·m, TqMin/Max 16bit)
 * D[19..20] iqref [15:0] (uint16, = iqref_A * 100 + 10000, 量程 -100A~+555A)
 * D[21..22] iqfdb [15:0] (uint16, = iqfdb_A * 100 + 10000, 来自 s_iq_fb_filt_q10)
 * D[23..24] Irms  [15:0] (uint16, = Irms_A * 100 + 10000, √((Id² + Iq²)/2) 真实 RMS)
 * D[25..26] MIT_T [15:0] (实际 I_q_ref[A] 过 Kt LUT → N·m, TqMin/Max 16bit)
 * D[27]     Vdc (uint8, 母线电压 V, 直接值)
 * D[28..29] Temp_D [15:0] (int16 LE, 驱动板温度 0.1°C)
 * D[30..31] Temp_M [15:0] (int16 LE, 电机温度 0.1°C)
 */
#define CAN_WLY_STATUS_FRAME_LEN 32

static void pack_status_frame(uint8_t *d) {
    float pos_rad = pos_int_to_rad(controller_eyou.real_position_out);
    float vel_rad_s = vel_motor_to_load_rad_s(controller_eyou.dtheta_mech);
    float tq_nm = tq_iq_to_nm(s_iq_fb_filt_q10);

    uint32_t p_int = float_to_uint(pos_rad, g_can_wly_lim.pos_min, g_can_wly_lim.pos_max, 24);
    uint16_t v_int = (uint16_t)float_to_uint(vel_rad_s, g_can_wly_lim.spd_min, g_can_wly_lim.spd_max, 16);
    uint16_t t_int = (uint16_t)float_to_uint(tq_nm, g_can_wly_lim.tq_min, g_can_wly_lim.tq_max, 16);

    uint16_t err1 = (uint16_t)(controller_eyou.ServoErrFlag.All_Flag & 0xFFFF);
    uint16_t err2 = (uint16_t)((controller_eyou.ServoErrFlag.All_Flag >> 16) & 0xFFFF);

    /* WARN: Bit0=MOS过温警告(90°C), Bit1=电机过温警告 */
    uint8_t  warn = 0;
    if (motorProValue.board_temp >= (int16_t)Threshold.TemBoradWarn) warn |= 0x01;
    if (motorProValue.motor_temp >= (int16_t)Threshold.TemMortorWarn) warn |= 0x02;

    /* Pact[23:0] */
    d[0] = p_int & 0xFF;
    d[1] = (p_int >> 8) & 0xFF;
    d[2] = (p_int >> 16) & 0xFF;
    /* Vact[15:0] */
    d[3] = v_int & 0xFF;
    d[4] = (v_int >> 8) & 0xFF;
    /* Tact[15:0] */
    d[5] = t_int & 0xFF;
    d[6] = (t_int >> 8) & 0xFF;
    /* Err1[15:0] */
    d[7] = err1 & 0xFF;
    d[8] = (err1 >> 8) & 0xFF;
    /* Err2[7:0] - 只占1字节! (协议表格修正) */
    d[9] = err2 & 0xFF;
    /* warn */
    d[10] = warn;

    /* STA */
    uint8_t sta = 0;
    if (controller_eyou.foc_run) sta |= 0x01;                       /* Bit0: 使能 */
    if (controller_eyou.ServoErrFlag.All_Flag) sta |= 0x02;         /* Bit1: 故障 */
    if (warn) sta |= 0x04;                                          /* Bit2: 警告 */
    if (controller_eyou.ServoState.Bit.PositionArrivedFlag ||
        controller_eyou.ServoState.Bit.SpeedArrivedFlag ||
        controller_eyou.ServoState.Bit.CurrentArrivedFlag) sta |= 0x08;  /* Bit3: 到达 */
    d[11] = sta;

    /* ===== 扩展字段: 指令 + 反馈细分 ===== */
    /* Pcmd 指令: 输出端 1°/1024 → rad → 24bit */
    float pos_ref_rad = pos_int_to_rad(controller_eyou.position_ref);
    uint32_t p_ref_u = float_to_uint(pos_ref_rad, g_can_wly_lim.pos_min, g_can_wly_lim.pos_max, 24);
    d[12] = p_ref_u & 0xFF;
    d[13] = (p_ref_u >> 8) & 0xFF;
    d[14] = (p_ref_u >> 16) & 0xFF;

    /* Vcmd 指令: velocity_ref (电机端 rpm*1024*GR) → 输出端 rad/s → 16bit */
    float vel_ref_rad_s = (float)controller_eyou.velocity_ref / (1024.0f * CAN_WLY_GR) * RPM_TO_RAD_S;
    uint16_t v_ref_u = (uint16_t)float_to_uint(vel_ref_rad_s, g_can_wly_lim.spd_min, g_can_wly_lim.spd_max, 16);
    d[15] = v_ref_u & 0xFF;
    d[16] = (v_ref_u >> 8) & 0xFF;

    /* Tcmd 指令: 上位机最近一次下发原始 N·m (无 LUT 反推, 量化误差最小) */
    uint16_t t_cmd_u = (uint16_t)float_to_uint(s_last_torque_cmd_nm, g_can_wly_lim.tq_min, g_can_wly_lim.tq_max, 16);
    d[17] = t_cmd_u & 0xFF;
    d[18] = (t_cmd_u >> 8) & 0xFF;

    /* iqref / iqfdb / Irms: A → uint16 = A * 100 + 10000
     * 量程: (0..65535 - 10000) / 100 = -100A ~ +555A */
    float iq_ref_A = (float)controller_eyou.I_q_ref / 1024.0f;
    uint16_t iq_ref_u = (uint16_t)(iq_ref_A * 100.0f + 10000.0f);
    d[19] = iq_ref_u & 0xFF;
    d[20] = (iq_ref_u >> 8) & 0xFF;

    float iq_fb_A = (float)s_iq_fb_filt_q10 / 1024.0f;
    uint16_t iq_fb_u = (uint16_t)(iq_fb_A * 100.0f + 10000.0f);
    d[21] = iq_fb_u & 0xFF;
    d[22] = (iq_fb_u >> 8) & 0xFF;

    /* Irms: √((Id² + Iq²)/2) 真实 RMS */
    float id_A = (float)controller_eyou.I_d / 1024.0f;
    float iq_A = (float)s_iq_fb_filt_q10 / 1024.0f;
    float irms_A = sqrtf((id_A * id_A + iq_A * iq_A) / 2.0f);
    uint16_t irms_u = (uint16_t)(irms_A * 100.0f + 10000.0f);
    d[23] = irms_u & 0xFF;
    d[24] = (irms_u >> 8) & 0xFF;

    /* MIT_T: 实际 I_q_ref (MIT 解算后含 Kp·err+Kd·err+t_ff, 已 clamp) → N·m
     * 与 D[17..18] Tcmd 同量纲, 可对比"上位机指令 vs 实际输出"
     * 非 MIT 模式下此字段即整机输出扭矩 (PI 控制器输出) */
    float iq_out_A = (float)controller_eyou.I_q_ref / 1024.0f;
    float mit_t_nm = can_wly_iA_to_Nm(iq_out_A);
    uint16_t mit_t_u = (uint16_t)float_to_uint(mit_t_nm, g_can_wly_lim.tq_min, g_can_wly_lim.tq_max, 16);
    d[25] = mit_t_u & 0xFF;
    d[26] = (mit_t_u >> 8) & 0xFF;

    /* Vdc: 母线电压 V, uint8 直接值 (量程 0~255V)
     * g_vdc_raw 是 16位 ADC 平均值 (2 次采样), 按 adc.c:499 公式:
     * V_bus = raw * 33 * 21 / 65535 / 10  (分压比 21:1, ADC 3.3V 满量程)
     * 简化: V_bus = raw * 0.01071... ≈ raw * 11 / 1024 */
    extern volatile uint32_t g_vdc_raw;
    uint16_t vdc_V = (uint16_t)((g_vdc_raw * 33UL * 21UL) / (65535UL * 10UL));
    uint8_t vdc_u8 = (vdc_V > 255) ? 255 : (uint8_t)vdc_V;
    d[27] = vdc_u8;

    /* Temp_D / Temp_M: int16 °C → 0.1°C (有符号, 负温度也正确) */
    int16_t temp_d = motorProValue.board_temp * 10;
    d[28] = temp_d & 0xFF;
    d[29] = (temp_d >> 8) & 0xFF;

    int16_t temp_m = motorProValue.motor_temp * 10;
    d[30] = temp_m & 0xFF;
    d[31] = (temp_m >> 8) & 0xFF;
}

static void send_status_frame(void) {
    uint8_t d[CAN_WLY_STATUS_FRAME_LEN];
    pack_status_frame(d);
    can_dbg_push(CAN_WLY_ID_STATUS_BASE + s_node_id, d, CAN_WLY_STATUS_FRAME_LEN, 1);
    if (fdcan_send(CAN_WLY_ID_STATUS_BASE + s_node_id, d, CAN_WLY_STATUS_FRAME_LEN) != HAL_OK) {
        s_tx_fail_count++;
    }
}

/* ========== 0x7FE 扩展状态帧 (16 字节, 兼容 H7 参考工程) ==========
 * D[0..1]  电流有效值 (0.01A, uint16 LE)
 * D[2..3]  速度 (0.1 输出端 rpm, int16 LE)
 * D[4..7]  位置 (0.001°输出端, int32 LE)
 * D[8..9]  电机温度 (0.1°C, int16 LE)
 * D[10..11] MOS 温度 (0.1°C, int16 LE)
 * D[12]    状态: Bit0=运行, Bit1=故障, Bit2=警告, Bit3=到达位
 * D[13..15] 保留 = 0
 */
static void send_ext_status_frame(void) {
    uint8_t d[16] = {0};

    /* 电流 RMS: |I_q_filt| / 1024 (A) × 100 → 0.01A
     * 用滤波值避免 1kHz 抽样混叠 PWM 纹波 */
    int32_t iq_abs = s_iq_fb_filt_q10;
    if (iq_abs < 0) iq_abs = -iq_abs;
    uint16_t i_rms_100 = (uint16_t)((iq_abs * 100) / 1024);

    /* 速度: dtheta_mech (电机端 rpm*1024*GR) → 0.1 输出端 rpm
       (折算: rpm*1024*GR / (1024*GR) * 10 = *10/(1024*GR)) */
    int16_t v_int = (int16_t)((controller_eyou.dtheta_mech * 10) / ((int32_t)(1024 * CAN_WLY_GR)));

    /* 位置: real_position_out (1°/1024) → 0.001° */
    int32_t p_int = (int32_t)(((int64_t)controller_eyou.real_position_out * 1000) / 1024);

    /* 温度: int16 °C → int16 0.1°C (有符号, 负温度也正确) */
    int16_t temp_motor = motorProValue.motor_temp * 10;
    int16_t temp_mos   = motorProValue.board_temp * 10;

    d[0] = i_rms_100 & 0xFF;
    d[1] = (i_rms_100 >> 8) & 0xFF;
    d[2] = v_int & 0xFF;
    d[3] = (v_int >> 8) & 0xFF;
    d[4] = p_int & 0xFF;
    d[5] = (p_int >> 8) & 0xFF;
    d[6] = (p_int >> 16) & 0xFF;
    d[7] = (p_int >> 24) & 0xFF;
    d[8] = temp_motor & 0xFF;
    d[9] = (temp_motor >> 8) & 0xFF;
    d[10] = temp_mos & 0xFF;
    d[11] = (temp_mos >> 8) & 0xFF;

    uint8_t sta = 0;
    if (controller_eyou.foc_run) sta |= 0x01;
    if (controller_eyou.ServoErrFlag.All_Flag) sta |= 0x02;
    if (motorProValue.board_temp >= (int16_t)Threshold.TemBoradWarn) sta |= 0x04;
    if (controller_eyou.ServoState.Bit.PositionArrivedFlag) sta |= 0x08;
    d[12] = sta;

    if (fdcan_send(CAN_WLY_ID_EXT_STATUS, d, 16) != HAL_OK) {
        s_tx_fail_count++;
    }
    can_dbg_push(CAN_WLY_ID_EXT_STATUS, d, 16, 1);
}

/* 在批量广播帧中查找与自身 ID 匹配的槽位, 返回槽内偏移; -1 表示不在列表中 */
static int32_t find_slot(const uint8_t *data, uint32_t len, uint32_t slot_size) {
    for (uint32_t off = 0; off + slot_size <= len; off += slot_size) {
        if (data[off + slot_size - 1] == s_node_id) return (int32_t)off;
    }
    return -1;
}

/* 0x200 速度指令: 3 字节/槽 = V_des[7:0], V_des[15:8], CANID */
static void handle_speed_cmd(const uint8_t *data, uint32_t len) {
    int32_t off = find_slot(data, len, 3);
    if (off < 0) return;
    uint16_t v_raw = (uint16_t)data[off] | ((uint16_t)data[off + 1] << 8);
    float v_rad_s = uint_to_float(v_raw, g_can_wly_lim.spd_min, g_can_wly_lim.spd_max, 16);
    controller_eyou.velocity_ref = vel_rad_s_to_int(v_rad_s);
    controller_eyou.controller_mode = PROFILE_VELOCITY_MOCE;
    send_status_frame();
}

/* 0x300 转矩指令: 3 字节/槽 = T_des[7:0], T_des[15:8], CANID */
static void handle_torque_cmd(const uint8_t *data, uint32_t len) {
    int32_t off = find_slot(data, len, 3);
    if (off < 0) return;
    uint16_t t_raw = (uint16_t)data[off] | ((uint16_t)data[off + 1] << 8);
    float t_nm = uint_to_float(t_raw, g_can_wly_lim.tq_min, g_can_wly_lim.tq_max, 16);
    int32_t iq = tq_nm_to_iq(t_nm);
    int32_t max_cur = (int32_t)controller_eyou.FlashData.MaxCurrent;
    if (iq >  max_cur) iq =  max_cur;
    else if (iq < -max_cur) iq = -max_cur;
    controller_eyou.I_q_ref = iq;
    s_last_torque_cmd_nm = t_nm;
    controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
    send_status_frame();
}

/* 0x400 位置指令: 6 字节/槽 = POS[23:0]+V_des[15:0]+CANID */
static void handle_position_cmd(const uint8_t *data, uint32_t len) {
    int32_t off = find_slot(data, len, 6);
    if (off < 0) return;
    uint32_t p_raw = (uint32_t)data[off] | ((uint32_t)data[off + 1] << 8) |
                     ((uint32_t)data[off + 2] << 16);
    uint16_t v_raw = (uint16_t)data[off + 3] | ((uint16_t)data[off + 4] << 8);
    float p_rad = uint_to_float(p_raw, g_can_wly_lim.pos_min, g_can_wly_lim.pos_max, 24);
    float v_rad_s = uint_to_float(v_raw, g_can_wly_lim.spd_min, g_can_wly_lim.spd_max, 16);
    controller_eyou.position_ref = pos_rad_to_int(p_rad);
    /* 速度限制: 取 CAN 指令与 MaxSpeed(100rpm) 中较小值 */
    {
        int32_t can_lim = vel_rad_s_to_int(fabsf(v_rad_s));
        int32_t ceiling = (int32_t)controller_eyou.FlashData.MaxSpeed;
        int32_t eff_lim = (can_lim < ceiling) ? can_lim : ceiling;
        controller_eyou.FlashData.Pid_PositionLimit = eff_lim;
        /* 同步梯形规划 v_max: 内部单位 → output rpm → LSB/tick */
        float rpm_out = (float)eff_lim / (1024.0f * CAN_WLY_GR);
        float vmax_lsb_tick = rpm_out * (6.0f * 1024.0f / 2500.0f);
        if (vmax_lsb_tick > 0.1f)
            controller_eyou.SmoothPosRef.v_max = vmax_lsb_tick;
    }
    controller_eyou.controller_mode = PROFILE_POSITION_MODE;
    send_status_frame();
}

/* 0x500 MIT指令: 12 字节/槽 = POS[23:0]+VEL[15:0]+T[15:0]+Kp[15:0]+Kd[15:0]+CANID
 * MIT 运算 (移植自 motor_h7 FOC.c:351):
 *   Iq_ref = Kp*(p_des - pos_cur) + Kd*(v_des - vel_cur) + t_ff
 * 运算在 FOC 主循环 (速度环周期) 执行, 此处仅解析并存入 controller */
static void handle_mit_cmd(const uint8_t *data, uint32_t len) {
    int32_t off = find_slot(data, len, 12);
    if (off < 0) return;
    uint32_t p_raw  = (uint32_t)data[off] | ((uint32_t)data[off + 1] << 8) |
                      ((uint32_t)data[off + 2] << 16);
    uint16_t v_raw  = (uint16_t)data[off + 3] | ((uint16_t)data[off + 4] << 8);
    uint16_t t_raw  = (uint16_t)data[off + 5] | ((uint16_t)data[off + 6] << 8);
    uint16_t kp_raw = (uint16_t)data[off + 7] | ((uint16_t)data[off + 8] << 8);
    uint16_t kd_raw = (uint16_t)data[off + 9] | ((uint16_t)data[off + 10] << 8);

    float p_des = uint_to_float(p_raw, g_can_wly_lim.pos_min, g_can_wly_lim.pos_max, 24);
    float v_des = uint_to_float(v_raw, g_can_wly_lim.spd_min, g_can_wly_lim.spd_max, 16);
    float t_nm  = uint_to_float(t_raw, g_can_wly_lim.tq_min, g_can_wly_lim.tq_max, 16);
    float t_ff  = can_wly_Nm_to_iA(t_nm);
    float kp    = uint_to_float(kp_raw, g_can_wly_lim.kp_min, g_can_wly_lim.kp_max, 16);
    float kd    = uint_to_float(kd_raw, g_can_wly_lim.kd_min, g_can_wly_lim.kd_max, 16);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    controller_eyou.mit_p_des = p_des;
    controller_eyou.mit_v_des = v_des;
    controller_eyou.mit_t_ff  = t_ff;
    controller_eyou.mit_kp    = kp;
    controller_eyou.mit_kd    = kd;
    controller_eyou.controller_mode = MIT_PD_MODE;
    s_last_torque_cmd_nm = t_nm;
    s_mit_timeout_cnt = MIT_TIMEOUT_MS;
    if (primask == 0U) {
        __enable_irq();
    }
    send_status_frame();
}

/* 0x600+ID SDO: 对象字典读(0x40)/写(0x23)
 * 读: 请求 D[0]=0x40, D[1..2]=idx, D[3]=subidx, 响应 0x580+ID, D[0]=0x60, D[4..7]=data
 * 写: 请求 D[0]=0x23, D[1..2]=idx, D[3]=subidx, D[4..7]=data, 响应 0x580+ID, D[0]=0x60, 回显
 * 失败响应 D[0]=0x80, D[1]=err
 */
static void sdo_pack_resp(uint8_t *d, uint8_t cmd, const uint8_t *req, const uint8_t *payload) {
    d[0] = cmd;
    d[1] = req[1];
    d[2] = req[2];
    d[3] = req[3];
    d[4] = payload[0];
    d[5] = payload[1];
    d[6] = payload[2];
    d[7] = payload[3];
}

/* 读: 把字典值填到 out[4]; 支持的索引返回 1 */
static uint8_t sdo_read_value(uint16_t idx, uint8_t subidx, uint8_t out[4]) {
    (void)subidx;
    uint8_t zero[4] = {0};
    memcpy(out, zero, 4);
    switch (idx) {
    case CAN_WLY_OD_POS_MIN:  float_to_bytes_le(g_can_wly_lim.pos_min, out); return 1;
    case CAN_WLY_OD_POS_MAX:  float_to_bytes_le(g_can_wly_lim.pos_max, out); return 1;
    case CAN_WLY_OD_SPD_MIN:  float_to_bytes_le(g_can_wly_lim.spd_min, out); return 1;
    case CAN_WLY_OD_SPD_MAX:  float_to_bytes_le(g_can_wly_lim.spd_max, out); return 1;
    case CAN_WLY_OD_TQ_MIN:   float_to_bytes_le(g_can_wly_lim.tq_min,  out); return 1;
    case CAN_WLY_OD_TQ_MAX:   float_to_bytes_le(g_can_wly_lim.tq_max,  out); return 1;
    case CAN_WLY_OD_KP_MIN:   float_to_bytes_le(g_can_wly_lim.kp_min,  out); return 1;
    case CAN_WLY_OD_KP_MAX:   float_to_bytes_le(g_can_wly_lim.kp_max,  out); return 1;
    case CAN_WLY_OD_KD_MIN:   float_to_bytes_le(g_can_wly_lim.kd_min,  out); return 1;
    case CAN_WLY_OD_KD_MAX:   float_to_bytes_le(g_can_wly_lim.kd_max,  out); return 1;
    case CAN_WLY_OD_SYNC_CYCLE: out[0] = 0xE8; out[1] = 0x03; return 1;  /* 1000 */
    case CAN_WLY_OD_DRI_POS_KP: {
        uint32_t v = controller_eyou.FlashData.Position_Kp;
        out[0] = v; out[1] = v >> 8; out[2] = v >> 16; out[3] = v >> 24; return 1;
    }
    case CAN_WLY_OD_DRI_SPD_KP: {
        uint32_t v = controller_eyou.FlashData.Speed_Kp;
        out[0] = v; out[1] = v >> 8; out[2] = v >> 16; out[3] = v >> 24; return 1;
    }
    case CAN_WLY_OD_DRI_SPD_KI: {
        uint32_t v = controller_eyou.FlashData.Speed_Ki;
        out[0] = v; out[1] = v >> 8; out[2] = v >> 16; out[3] = v >> 24; return 1;
    }
    case CAN_WLY_OD_NODE_ID: out[0] = s_node_id; return 1;
    case CAN_WLY_OD_AUTO_REPORT: out[0] = s_auto_report ? 2 : 0; return 1;
    case CAN_WLY_OD_TEST_FREQ: {
        uint32_t v = s_test_freq_hz;
        out[0] = v; out[1] = v >> 8; out[2] = v >> 16; out[3] = v >> 24; return 1;
    }
    case CAN_WLY_OD_TEST_AMPL: {
        uint32_t v = s_test_ampl_q10;
        out[0] = v; out[1] = v >> 8; out[2] = v >> 16; out[3] = v >> 24; return 1;
    }
    default: return 0;
    }
}

/* 写: 返回 1 成功, 0 不支持 */
static uint8_t sdo_write_value(uint16_t idx, uint8_t subidx, const uint8_t *in) {
    (void)subidx;
    switch (idx) {
    case CAN_WLY_OD_POS_MIN: {
        g_can_wly_lim.pos_min = bytes_to_float_le(in);
        const float rad_to_lsb = 180.0f * 1024.0f / (float)M_PI;
        controller_eyou.FlashData.MinPositionLimit = (int32_t)(g_can_wly_lim.pos_min * rad_to_lsb);
        controller_eyou.FlashData.PositionLimitFlag = 50;
        controller_eyou.UserDataSaveFlag = 1;
        return 1;
    }
    case CAN_WLY_OD_POS_MAX: {
        g_can_wly_lim.pos_max = bytes_to_float_le(in);
        const float rad_to_lsb = 180.0f * 1024.0f / (float)M_PI;
        controller_eyou.FlashData.MaxPositionLimit = (int32_t)(g_can_wly_lim.pos_max * rad_to_lsb);
        controller_eyou.FlashData.PositionLimitFlag = 50;
        controller_eyou.UserDataSaveFlag = 1;
        return 1;
    }
    case CAN_WLY_OD_SPD_MIN: g_can_wly_lim.spd_min = bytes_to_float_le(in); return 1;
    case CAN_WLY_OD_SPD_MAX: g_can_wly_lim.spd_max = bytes_to_float_le(in); return 1;
    case CAN_WLY_OD_TQ_MIN:  g_can_wly_lim.tq_min  = bytes_to_float_le(in); return 1;
    case CAN_WLY_OD_TQ_MAX:  g_can_wly_lim.tq_max  = bytes_to_float_le(in); return 1;
    case CAN_WLY_OD_KP_MIN:  g_can_wly_lim.kp_min  = bytes_to_float_le(in); return 1;
    case CAN_WLY_OD_KP_MAX:  g_can_wly_lim.kp_max  = bytes_to_float_le(in); return 1;
    case CAN_WLY_OD_KD_MIN:  g_can_wly_lim.kd_min  = bytes_to_float_le(in); return 1;
    case CAN_WLY_OD_KD_MAX:  g_can_wly_lim.kd_max  = bytes_to_float_le(in); return 1;
    case CAN_WLY_OD_NODE_ID: {
        uint8_t new_id = in[0];
        if (new_id >= CAN_WLY_ID_MIN && new_id <= CAN_WLY_ID_MAX) {
            s_node_id = new_id;
            /* 保存到 Flash (temp4 低字节, 避免与 Rs/temp1 冲突) */
            controller_eyou.FlashData.temp4 = (controller_eyou.FlashData.temp4 & 0xFFFFFF00) | new_id;
            controller_eyou.UserDataSaveFlag = 1;
            return 1;
        }
        return 0;
    }
    case CAN_WLY_OD_CALI: {
        /* 0x2F01 sub=1 data[0]=1: 触发学零位 (主循环执行, ISR 只设标志)
         * CAN 帧: 60x 23 01 2F 01 01 FF FF FF */
        if (subidx == 0x01 && in[0] == 0x01) {
            extern volatile uint8_t g_can_cali_request;
            g_can_cali_request = 1;
            return 1;
        }
        return 0;
    }
    case CAN_WLY_OD_AUTO_REPORT: {
        /* 0x2F05 测试命令: 0=停 0x7FD 流, 1=启 0x7FD 流 (单频注入), 2=开 1ms 0x7FE 周报
         * 三条命令独立, 互不影响对方状态 */
        uint8_t cmd = in[0];
        if (cmd == 0) {
            s_test_active = 0;
        } else if (cmd == 1) {
            /* 启动单频注入: phase_step = freq / FOC_FREQUENCY * 2^32
             * FOC_FREQUENCY = 10000Hz, 直接用 (uint64) 算, 防 32 位溢出 */
            uint64_t step = ((uint64_t)s_test_freq_hz << 32) / 10000ULL;
            s_test_phase_step = (uint32_t)step;
            s_test_phase_acc = 0;
            s_test_active = 1;
        } else if (cmd == 2) {
            s_auto_report = 1;
        }
        return 1;
    }
    case CAN_WLY_OD_TEST_FREQ: {
        uint32_t v = (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
                     ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
        if (v == 0) v = 1;                  /* 防 0 频率 */
        if (v > 5000) v = 5000;             /* 限 < Nyquist (FOC=10kHz, Nyquist=5kHz) */
        s_test_freq_hz = v;
        return 1;
    }
    case CAN_WLY_OD_TEST_AMPL: {
        uint32_t v = (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
                     ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
        if (v > 30 * 1024U) v = 30 * 1024U; /* 限 30A 防过流 */
        s_test_ampl_q10 = v;
        return 1;
    }
    case CAN_WLY_OD_SYNC_CYCLE:
    case CAN_WLY_OD_DRI_POS_KP:
    case CAN_WLY_OD_DRI_SPD_KP:
    case CAN_WLY_OD_DRI_SPD_KI:
        /* 只读或驱动层参数, 暂不支持在线写 */
        return 0;
    default: return 0;
    }
}

static void handle_sdo_frame(const uint8_t *req, uint32_t len) {
    if (len < 8) return;
    uint8_t cmd = req[0];
    uint16_t idx = (uint16_t)req[1] | ((uint16_t)req[2] << 8);
    uint8_t subidx = req[3];
    uint8_t resp[8] = {0};

    if (cmd == CAN_WLY_SDO_READ_REQ) {
        uint8_t payload[4] = {0};
        if (sdo_read_value(idx, subidx, payload)) {
            sdo_pack_resp(resp, CAN_WLY_SDO_ACK, req, payload);
        } else {
            sdo_pack_resp(resp, CAN_WLY_SDO_ERR, req, payload);
            resp[1] = 0x06;  /* err code: object does not exist */
        }
    } else if (cmd == CAN_WLY_SDO_WRITE_REQ) {
        if (sdo_write_value(idx, subidx, &req[4])) {
            sdo_pack_resp(resp, CAN_WLY_SDO_ACK, req, &req[4]);
        } else {
            uint8_t payload[4] = {0};
            sdo_pack_resp(resp, CAN_WLY_SDO_ERR, req, payload);
            resp[1] = 0x06;
        }
    } else {
        return;
    }
    if (fdcan_send(CAN_WLY_ID_SDO_RSP_BASE + s_node_id, resp, 8) != HAL_OK) {
        s_tx_fail_count++;
    }
    can_dbg_push(CAN_WLY_ID_SDO_RSP_BASE + s_node_id, resp, 8, 1);
}

static void handle_ctrl_frame(const uint8_t *data, uint32_t len) {
    if (len < 8) return;
    for (int i = 0; i < 7; i++) {
        if (data[i] != 0xFF) return;
    }
    switch (data[7]) {
    case CAN_WLY_CTRL_ENABLE:
        /* 启动前检查: 刹车进行中 / 故障未清 → 拒绝启动 */
        if (fault_brake_is_active()) {
            return;  /* 刹车状态机进行中, 忽略启动 */
        }
        if (controller_eyou.ServoErrFlag.All_Flag != 0) {
            return;  /* 有未清故障, 必须先 CLR_ERR */
        }
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
        break;
    case CAN_WLY_CTRL_DISABLE:
        /* 直接进入安全停机状态机 (coast→brake→shutdown).
         * 不能用 HAL_Delay: 此函数在 FDCAN1_IT0 ISR (优先级 6) 里执行,
         * 会屏蔽 TIM7 (优先级 15) 的 HAL Timebase 导致 HAL_Delay 死锁,
         * 同时阻塞 CAN FIFO 引发溢出/BusOff. */
        fault_safe_shutdown();
        break;
    case CAN_WLY_CTRL_SET_ZERO:
        /* 将当前机械位置设为零点 (PHU 接口) */
        controller_eyou.controller_mode = HOMING_MODE;
        Reset_objReset_Output_Encoder(1);
        controller_eyou.UserDataSaveFlag = 1;
        Reset_objReset_Output_Encoder(0);
        break;
    case CAN_WLY_CTRL_CLR_ERR:
        ClearFaults(1);
        break;
    default:
        return;
    }
    send_status_frame();
}

/* ========== 顶层 RX 分发 (覆盖 fdcan.c 的 weak 回调) ========== */
static volatile uint32_t s_rx_frame_cnt = 0;
volatile uint8_t g_can_rx_debug = 0;

/* 环形缓冲: ISR 写, main 循环 print (避免在 ISR 里调 printf 死锁) */
#define CAN_DBG_BUF_SIZE 8
typedef struct {
    uint32_t id;
    uint8_t  data[16];
    uint8_t  len;
    uint8_t  dir;  /* 0=RX, 1=TX */
} can_dbg_entry_t;
static volatile can_dbg_entry_t s_can_dbg_buf[CAN_DBG_BUF_SIZE];
static volatile uint8_t s_can_dbg_wr = 0;
static volatile uint8_t s_can_dbg_rd = 0;

static void can_dbg_push(uint32_t id, const uint8_t *data, uint32_t len, uint8_t dir) {
    if (!g_can_rx_debug) return;
    uint8_t next = (s_can_dbg_wr + 1) & (CAN_DBG_BUF_SIZE - 1);
    if (next == s_can_dbg_rd) return;
    s_can_dbg_buf[s_can_dbg_wr].id  = id;
    s_can_dbg_buf[s_can_dbg_wr].len = (uint8_t)((len > 16) ? 16 : len);
    s_can_dbg_buf[s_can_dbg_wr].dir = dir;
    for (uint8_t i = 0; i < s_can_dbg_buf[s_can_dbg_wr].len; i++)
        s_can_dbg_buf[s_can_dbg_wr].data[i] = data[i];
    s_can_dbg_wr = next;
}

void can_wly_dbg_poll(void) {
    while (s_can_dbg_rd != s_can_dbg_wr) {
        const can_dbg_entry_t *e = (const can_dbg_entry_t *)&s_can_dbg_buf[s_can_dbg_rd];
        printf("[CAN %s] ID=0x%03X len=%u D=",
               e->dir ? "TX" : "RX", (unsigned int)e->id, e->len);
        for (uint8_t i = 0; i < e->len && i < 16; i++) printf("%02X ", e->data[i]);
        printf("\r\n");
        s_can_dbg_rd = (s_can_dbg_rd + 1) & (CAN_DBG_BUF_SIZE - 1);
    }
}

void fdcan_rx_user(uint32_t id, const uint8_t *data, uint32_t len) {
    s_rx_frame_cnt++;
    can_dbg_push(id, data, len, 0);

    /* CAN 调试通道旁路 (0x7E0~0x7EF): 不喂万里扬看门狗 */
    if (id >= CAN_DBG_ID_CMD && id <= 0x7EF) {
        can_debug_rx_isr(id, data, len);
        return;
    }

#if (CAN_PROTOCOL_SEL == CAN_PROTO_CYBEAST)
    /* 守护兽模式: 走 can_cybeast 分发 */
    extern void can_cybeast_rx_dispatch(uint32_t id, const uint8_t *data, uint32_t len);
    can_cybeast_rx_dispatch(id, data, len);
#else
    s_can_timeout_cnt = CAN_TIMEOUT_MS;
    if (!s_can_timeout_enabled) s_can_timeout_enabled = 1;
    /* 广播查询: 所有从站回 0x100+ID */
    if (id == CAN_WLY_ID_QUERY_BCAST) {
        send_status_frame();
        return;
    }
    /* 批量指令: 需扫描整帧匹配 s_node_id */
    if (id == CAN_WLY_ID_SPEED)    { handle_speed_cmd(data, len);    return; }
    if (id == CAN_WLY_ID_TORQUE)   { handle_torque_cmd(data, len);   return; }
    if (id == CAN_WLY_ID_POSITION) { handle_position_cmd(data, len); return; }
    if (id == CAN_WLY_ID_MIT)      { handle_mit_cmd(data, len);      return; }

    /* 点对点: 仅处理本节点 ID */
    uint32_t cat = id & 0x780;  /* 高位类别 */
    uint32_t nid = id & 0x07F;
    if (nid != s_node_id) return;

    switch (cat) {
    case CAN_WLY_ID_CTRL_BASE:
        handle_ctrl_frame(data, len);
        break;
    case CAN_WLY_ID_SDO_REQ_BASE:
        handle_sdo_frame(data, len);
        break;
    case CAN_WLY_ID_QUERY_BCAST:
        /* 0x80+ID 单播查询: 与 motor_h7 兼容 */
        send_status_frame();
        break;
    default:
        break;
    }
#endif
}

void can_wly_init(void) {
    /* 从 Flash 恢复节点 ID (存储在 FlashData.temp4 低字节, 避免与 Rs/temp1 冲突) */
    uint8_t saved_id = (uint8_t)(controller_eyou.FlashData.temp4 & 0xFF);
    if (saved_id >= CAN_WLY_ID_MIN && saved_id <= CAN_WLY_ID_MAX) {
        s_node_id = saved_id;
    } else {
        s_node_id = CAN_WLY_ID_DEFAULT;
    }

    /* 从 Flash 恢复过载保护参数 (temp4 高 3 字节) */
    uint8_t ol_a = (uint8_t)((controller_eyou.FlashData.temp4 >> 8)  & 0xFF);
    uint8_t ol_w = (uint8_t)((controller_eyou.FlashData.temp4 >> 16) & 0xFF);
    uint8_t ol_s = (uint8_t)((controller_eyou.FlashData.temp4 >> 24) & 0xFF);
    if (ol_a > 0 && ol_w > 0 && ol_s > ol_w) {
        g_overload_current_A = ol_a;
        g_overload_warn_s    = ol_w;
        g_overload_stop_s    = ol_s;
    }

    /* 上电同步: 协议位置量程 → Flash 软限位, 让 motorOverPosCheck 用同一套范围 */
    const float rad_to_lsb = 180.0f * 1024.0f / (float)M_PI;
    controller_eyou.FlashData.MinPositionLimit = (int32_t)(g_can_wly_lim.pos_min * rad_to_lsb);
    controller_eyou.FlashData.MaxPositionLimit = (int32_t)(g_can_wly_lim.pos_max * rad_to_lsb);
    controller_eyou.FlashData.PositionLimitFlag = 50;
}

uint8_t can_wly_get_node_id(void) { return s_node_id; }
void can_wly_set_node_id(uint8_t id) {
    if (id >= CAN_WLY_ID_MIN && id <= CAN_WLY_ID_MAX) s_node_id = id;
}

uint32_t can_wly_get_tx_fail_count(void) { return s_tx_fail_count; }

/* 1ms tick: 自动上报 + CAN 超时保护 + 位置到达上报 */
void can_wly_tick_1ms(void) {
    /* 主动上报: 对齐 motor_h7 bDynamMode, 1ms 发 0x7FE 扩展状态帧 (16B) */
    if (s_auto_report) send_ext_status_frame();

    /* 位置到达: 上升沿触发一次 0x7FE 扩展状态帧 */
    static uint8_t pos_arrived_last = 0;
    uint8_t pos_arrived_now = controller_eyou.ServoState.Bit.PositionArrivedFlag;
    if (pos_arrived_now && !pos_arrived_last) {
        send_ext_status_frame();
    }
    pos_arrived_last = pos_arrived_now;

    if (s_can_timeout_enabled && !g_can_timeout_force_disable && s_can_timeout_cnt > 0) {
        if (--s_can_timeout_cnt == 0) {
            controller_eyou.ServoErrFlag.Bit.CommunicateErr = 1;
        }
    }

    if (!g_can_timeout_force_disable &&
        controller_eyou.controller_mode == MIT_PD_MODE &&
        s_mit_timeout_cnt > 0) {
        if (--s_mit_timeout_cnt == 0) {
            controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
            controller_eyou.I_q_ref = 0;
            controller_eyou.I_d_ref = 0;
            controller_eyou.ServoErrFlag.Bit.CommunicateErr = 1;
        }
    }
}

/* ========== 0x7FD 电流带宽测试逐拍数据流 (FOC ISR 直接调用) ==========
 * cmd=1 启动后:
 *   - pre()  在 PID 之前累加相位, 返回 ampl*sin(phase) 的 Q10 注入信号给 I_q_ref_filterd
 *   - post() 在 set_phase_voltage 之后, 打包 (Iq_ref_filterd, I_q) 立即发 0x7FD
 * 不活跃时 pre 返回 0, post 直返
 * 协议格式: byte[0..3] = (int32)(I_q_ref_filterd_A * 1000) + 50000   LSB first
 *           byte[4..7] = (int32)(I_q_A             * 1000) + 50000   LSB first
 * 量程 ±50A (uint32 装下偏置后值, 超界绕环失真 — 0x2F07 已限 30A) */

#include "foc_kernel.h"  /* get_sincos_value */

int16_t can_wly_test_isr_pre(void) {
    if (!s_test_active) return 0;
    uint16_t angle = (uint16_t)(s_test_phase_acc >> 16);
    Trig_Components sc = get_sincos_value((int32_t)angle);
    /* sc.hSin Q15, ampl Q10 -> ((Q10 * Q15) >> 15) = Q10 */
    int32_t inj = ((int32_t)s_test_ampl_q10 * sc.hSin) >> 15;
    s_test_phase_acc += s_test_phase_step;
    if (inj >  32767) inj =  32767;
    if (inj < -32768) inj = -32768;
    return (int16_t)inj;
}

void can_wly_test_isr_post(int32_t iq_ref_filterd, int32_t iq_fb) {
    if (!s_test_active) return;
    /* I (Q10) -> A * 1000 = Q10 * 1000 / 1024
     * 整数化: (iq_q10 * 125) / 128, 误差 < 0.025% (1000/1024 ≈ 0.9766, 125/128=0.9766) */
    int32_t ref_x1000 = (iq_ref_filterd * 125) / 128;
    int32_t fb_x1000  = (iq_fb          * 125) / 128;
    uint32_t ref_u = (uint32_t)(ref_x1000 + 50000);
    uint32_t fb_u  = (uint32_t)(fb_x1000  + 50000);
    uint8_t d[8];
    d[0] = (uint8_t)(ref_u);       d[1] = (uint8_t)(ref_u >> 8);
    d[2] = (uint8_t)(ref_u >> 16); d[3] = (uint8_t)(ref_u >> 24);
    d[4] = (uint8_t)(fb_u);        d[5] = (uint8_t)(fb_u >> 8);
    d[6] = (uint8_t)(fb_u >> 16);  d[7] = (uint8_t)(fb_u >> 24);
    if (fdcan_send(CAN_WLY_ID_TEST_RESULT, d, 8) != HAL_OK) {
        s_test_tx_fail_cnt++;
    } else {
        s_test_tx_ok_cnt++;
    }
}

/* ========== 串口 dbg 命令接口 (模拟 CAN SDO 写) ========== */

uint32_t can_wly_get_test_freq(void) {
    return s_test_freq_hz;
}

void can_wly_set_test_freq(uint32_t hz) {
    if (hz == 0) hz = 1;
    if (hz > 5000) hz = 5000;
    s_test_freq_hz = hz;
}

uint32_t can_wly_get_test_ampl(void) {
    return s_test_ampl_q10;
}

void can_wly_set_test_ampl(uint32_t q10) {
    if (q10 > 30 * 1024U) q10 = 30 * 1024U;
    s_test_ampl_q10 = q10;
}

void can_wly_test_start(void) {
    uint64_t step = ((uint64_t)s_test_freq_hz << 32) / 10000ULL;
    s_test_phase_step = (uint32_t)step;
    s_test_phase_acc = 0;
    s_test_tx_ok_cnt = 0;
    s_test_tx_fail_cnt = 0;
    s_test_active = 1;
}

void can_wly_test_stop(void) {
    s_test_active = 0;
}

uint32_t can_wly_get_test_tx_ok(void) {
    return s_test_tx_ok_cnt;
}

uint32_t can_wly_get_test_tx_fail(void) {
    return s_test_tx_fail_cnt;
}

/* ========== Iq 反馈一阶 LPF (仅供对外上报) ==========
 * y[n] = y[n-1] + (x[n] - y[n-1]) >> 3,  α = 1/8
 * 10kHz 采样下 fc = fs * α / (2π) ≈ 199Hz, 远高于速度环带宽 (46Hz)
 * 不会掩盖真实扭振; 同时压住 PWM 纹波 / ADC 量化, 让 1kHz CAN 抽样不再混叠.
 * 仅给 0x100 / 0x7FE 状态帧使用; PI 反馈/BEMF 解耦/死区补偿/故障检测仍用原始 I_q. */
void can_wly_iq_fb_filter_update(int32_t iq_q10) {
    s_iq_fb_filt_q10 += (iq_q10 - s_iq_fb_filt_q10) >> 3;
}

int32_t can_wly_iq_fb_get(void) {
    return s_iq_fb_filt_q10;
}
