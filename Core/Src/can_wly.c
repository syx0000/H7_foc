/**
 * @file    can_wly.c
 * @brief   万里扬FDCAN通信协议V1.7 - 从站实现
 */
#include "can_wly.h"
#include "fdcan.h"
#include "foc_api.h"
#include "foc_controller.h"
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

/* 发送失败计数器 (调试用) */
static uint32_t s_tx_fail_count = 0;

/* CAN 超时保护：收到有效帧时重置，1ms 递减，归零停机 */
#define CAN_TIMEOUT_MS 200
static volatile uint16_t s_can_timeout_cnt = 0;
static uint8_t s_can_timeout_enabled = 0;
uint8_t g_can_timeout_force_disable = 1;  /* 调试期间默认关闭, 联调时改回 0 */

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
/* 速度: dtheta_mech_out (输出端 rpm*1024, 不含GR) -> rad/s (反馈路径) */
static float vel_out_to_rad_s(int32_t vel_out) {
    float rpm_out = (float)vel_out / 1024.0f;
    return rpm_out * RPM_TO_RAD_S;
}
/* 速度: 输出端 rad/s -> velocity_ref (rpm*1024*GR) (指令路径) */
static int32_t vel_rad_s_to_int(float rad_s) {
    float rpm_out = rad_s / RPM_TO_RAD_S;
    return (int32_t)(rpm_out * 1024.0f * CAN_WLY_GR);
}
/* 转矩: q轴电流(Q10) -> N·m */
static float tq_iq_to_nm(int32_t iq_q10) {
    return ((float)iq_q10 / 1024.0f) * g_can_wly_kt_out;
}
/* 转矩: N·m -> q轴电流(Q10) */
static int32_t tq_nm_to_iq(float nm) {
    if (g_can_wly_kt_out == 0.0f) return 0;
    return (int32_t)(nm / g_can_wly_kt_out * 1024.0f);
}

/* ========== 0x100+ID 状态帧 (12 字节) ==========
 * D[0..2] POS[23:0] (float->定点, 按 PosMin/Max)
 * D[3..4] VEL[15:0] (float->定点, 按 SpdMin/Max)
 * D[5..6] T[15:0]   (float->定点, 按 TqMin/Max)
 * D[7..8] ERR1[15:0]
 * D[9]    ERR2
 * D[10]   WARN
 * D[11]   STA (Bit0=使能, Bit1=故障, Bit2=警告)
 */
static void pack_status_frame(uint8_t *d) {
    float pos_rad = pos_int_to_rad(controller_eyou.real_position_out);
    float vel_rad_s = vel_out_to_rad_s(controller_eyou.dtheta_mech_out);
    float tq_nm = tq_iq_to_nm(controller_eyou.I_q);

    uint32_t p_int = float_to_uint(pos_rad, g_can_wly_lim.pos_min, g_can_wly_lim.pos_max, 24);
    uint16_t v_int = (uint16_t)float_to_uint(vel_rad_s, g_can_wly_lim.spd_min, g_can_wly_lim.spd_max, 16);
    uint16_t t_int = (uint16_t)float_to_uint(tq_nm, g_can_wly_lim.tq_min, g_can_wly_lim.tq_max, 16);

    uint16_t err1 = (uint16_t)(controller_eyou.ServoErrFlag.All_Flag & 0xFFFF);
    uint8_t  err2 = (uint8_t)((controller_eyou.ServoErrFlag.All_Flag >> 16) & 0xFF);

    /* WARN: Bit0=MOS过温警告(90°C), Bit1=电机过温警告 */
    uint8_t  warn = 0;
    if (motorProValue.board_temp >= (int8_t)Threshold.TemBoradWarn) warn |= 0x01;
    if (motorProValue.motor_temp >= (int8_t)Threshold.TemBoradWarn) warn |= 0x02;

    d[0] = p_int & 0xFF;
    d[1] = (p_int >> 8) & 0xFF;
    d[2] = (p_int >> 16) & 0xFF;
    d[3] = v_int & 0xFF;
    d[4] = (v_int >> 8) & 0xFF;
    d[5] = t_int & 0xFF;
    d[6] = (t_int >> 8) & 0xFF;
    d[7] = err1 & 0xFF;
    d[8] = (err1 >> 8) & 0xFF;
    d[9] = err2;
    d[10] = warn;

    uint8_t sta = 0;
    if (controller_eyou.foc_run) sta |= 0x01;                       /* Bit0: 使能 */
    if (controller_eyou.ServoErrFlag.All_Flag) sta |= 0x02;         /* Bit1: 故障 */
    if (warn) sta |= 0x04;                                          /* Bit2: 警告 */
    if (controller_eyou.ServoState.Bit.PositionArrivedFlag ||
        controller_eyou.ServoState.Bit.SpeedArrivedFlag ||
        controller_eyou.ServoState.Bit.CurrentArrivedFlag) sta |= 0x08;  /* Bit3: 到达 */
    d[11] = sta;
}

static void send_status_frame(void) {
    uint8_t d[12];
    pack_status_frame(d);
    can_dbg_push(CAN_WLY_ID_STATUS_BASE + s_node_id, d, 12, 1);
    if (fdcan_send(CAN_WLY_ID_STATUS_BASE + s_node_id, d, 12) != HAL_OK) {
        s_tx_fail_count++;
    }
}

/* ========== 0x7FE 扩展状态帧 (16 字节, 兼容 H7 参考工程) ==========
 * D[0..1]  电流有效值 (0.01A, uint16 LE)
 * D[2..3]  速度 (0.1 rpm 输出端, int16 LE)
 * D[4..7]  位置 (0.001°输出端, int32 LE)
 * D[8..9]  电机温度 (0.1°C, int16 LE)
 * D[10..11] MOS 温度 (0.1°C, int16 LE)
 * D[12]    状态: Bit0=运行, Bit1=故障, Bit2=警告, Bit3=到达位
 * D[13..15] 保留 = 0
 */
static void send_ext_status_frame(void) {
    uint8_t d[16] = {0};

    /* 电流 RMS: |I_q| / 1024 (A) × 100 → 0.01A */
    int32_t iq_abs = controller_eyou.I_q;
    if (iq_abs < 0) iq_abs = -iq_abs;
    uint16_t i_rms_100 = (uint16_t)((iq_abs * 100) / 1024);

    /* 速度: dtheta_mech_out (rpm×1024 输出端) → 0.1 rpm */
    int16_t v_int = (int16_t)((controller_eyou.dtheta_mech_out * 10) / 1024);

    /* 位置: real_position_out (1°/1024) → 0.001° */
    int32_t p_int = (int32_t)(((int64_t)controller_eyou.real_position_out * 1000) / 1024);

    /* 温度: int8 °C → int16 0.1°C (有符号, 负温度也正确) */
    int16_t temp_motor = (int16_t)motorProValue.motor_temp * 10;
    int16_t temp_mos   = (int16_t)motorProValue.board_temp * 10;

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
    if (motorProValue.board_temp >= (int8_t)Threshold.TemBoradWarn) sta |= 0x04;
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
    if (iq > INC_PID_SPEED_LIMIT) iq = INC_PID_SPEED_LIMIT;
    else if (iq < -INC_PID_SPEED_LIMIT) iq = -INC_PID_SPEED_LIMIT;
    controller_eyou.I_q_ref = iq;
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

    controller_eyou.mit_p_des = uint_to_float(p_raw, g_can_wly_lim.pos_min, g_can_wly_lim.pos_max, 24);
    controller_eyou.mit_v_des = uint_to_float(v_raw, g_can_wly_lim.spd_min, g_can_wly_lim.spd_max, 16);
    controller_eyou.mit_t_ff  = uint_to_float(t_raw, g_can_wly_lim.tq_min, g_can_wly_lim.tq_max, 16) / g_can_wly_kt_out;
    controller_eyou.mit_kp    = uint_to_float(kp_raw, g_can_wly_lim.kp_min, g_can_wly_lim.kp_max, 16);
    controller_eyou.mit_kd    = uint_to_float(kd_raw, g_can_wly_lim.kd_min, g_can_wly_lim.kd_max, 16);

    controller_eyou.controller_mode = MIT_PD_MODE;
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
    case CAN_WLY_OD_AUTO_REPORT: {
        /* 0x2F05: 主动上报模式 (0=关闭, 2=开启 1ms 周期上报) */
        s_auto_report = (in[0] == 2) ? 1 : 0;
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
        controller_eyou.I_q_ref = 0;
        controller_eyou.velocity_ref = 0;
        controller_eyou.position_ref = controller_eyou.real_position_out;
        controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
        controller_eyou.foc_run = 2;
        TIM1->CCER |= 0x0555u;
        break;
    case CAN_WLY_CTRL_DISABLE:
        /* 高速/大电流保护: 禁止直接停机, 先强制减速到安全值 */
        {
            int32_t spd_abs = controller_eyou.dtheta_mech;
            if (spd_abs < 0) spd_abs = -spd_abs;
            int32_t iq_abs = controller_eyou.I_q;
            if (iq_abs < 0) iq_abs = -iq_abs;

            /* 阈值: 载端 10rpm (256000) 或 15A (15360) */
            if (spd_abs > 256000 || iq_abs > 15360) {
                /* 强制切速度模式, 减速到 0 */
                controller_eyou.controller_mode = PROFILE_VELOCITY_MOCE;
                controller_eyou.velocity_ref = 0;
                controller_eyou.I_q_ref = 0;
                /* 等待 300ms 让速度环减速（保守） */
                HAL_Delay(300);
            }
        }

        /* 斜坡停机: 先清指令让电流环自然衰减, 再触发主动刹车 */
        controller_eyou.I_q_ref = 0;
        controller_eyou.velocity_ref = 0;
        controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
        /* 延迟 30ms 让电流环 PID 把电流降到接近 0（保守加长） */
        HAL_Delay(30);
        controller_eyou.foc_run = 0;
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
}

void can_wly_init(void) {
    /* 从 Flash 恢复节点 ID (存储在 FlashData.temp4 低字节, 避免与 Rs/temp1 冲突) */
    uint8_t saved_id = (uint8_t)(controller_eyou.FlashData.temp4 & 0xFF);
    if (saved_id >= CAN_WLY_ID_MIN && saved_id <= CAN_WLY_ID_MAX) {
        s_node_id = saved_id;
    } else {
        s_node_id = CAN_WLY_ID_DEFAULT;
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
}
