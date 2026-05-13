/**
 * @file    can_wly.c
 * @brief   万里扬FDCAN通信协议V1.7 - 从站实现
 */
#include "can_wly.h"
#include "fdcan.h"
#include "foc_api.h"
#include "foc_controller.h"
#include "foc_bsp.h"
#include <string.h>
#include <math.h>

/* ========== 内部状态 ========== */
static uint8_t s_node_id = CAN_WLY_ID_DEFAULT;

can_wly_limits_t g_can_wly_lim = {
    .pos_min = -5.0f,  .pos_max = 5.0f,
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

/* ========== 访问全局控制器 ========== */
extern ControllerStruct controller_eyou;

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
/* 速度: 内部电机端 (rpm*1024*GR) -> 输出端 rad/s */
static float vel_int_to_rad_s(int32_t vel_int) {
    float rpm_out = (float)vel_int / (1024.0f * CAN_WLY_GR);
    return rpm_out * RPM_TO_RAD_S;
}
/* 速度: 输出端 rad/s -> 内部电机端 (rpm*1024*GR) */
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
    float vel_rad_s = vel_int_to_rad_s(controller_eyou.dtheta_mech_out);
    float tq_nm = tq_iq_to_nm(controller_eyou.I_q);

    uint32_t p_int = float_to_uint(pos_rad, g_can_wly_lim.pos_min, g_can_wly_lim.pos_max, 24);
    uint16_t v_int = (uint16_t)float_to_uint(vel_rad_s, g_can_wly_lim.spd_min, g_can_wly_lim.spd_max, 16);
    uint16_t t_int = (uint16_t)float_to_uint(tq_nm, g_can_wly_lim.tq_min, g_can_wly_lim.tq_max, 16);

    uint16_t err1 = (uint16_t)(controller_eyou.ServoErrFlag.All_Flag & 0xFFFF);
    uint8_t  err2 = (uint8_t)((controller_eyou.ServoErrFlag.All_Flag >> 16) & 0xFF);
    uint8_t  warn = 0;

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
    if (controller_eyou.foc_run) sta |= 0x01;
    if (controller_eyou.ServoErrFlag.All_Flag) sta |= 0x02;
    if (warn) sta |= 0x04;
    d[11] = sta;
}

static void send_status_frame(void) {
    uint8_t d[12];
    pack_status_frame(d);
    if (fdcan_send(CAN_WLY_ID_STATUS_BASE + s_node_id, d, 12) != HAL_OK) {
        s_tx_fail_count++;
    }
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
    controller_eyou.I_q_ref = tq_nm_to_iq(t_nm);
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
    controller_eyou.FlashData.Pid_PositionLimit = vel_rad_s_to_int(fabsf(v_rad_s));
    controller_eyou.controller_mode = PROFILE_POSITION_MODE;
    send_status_frame();
}

/* 0x500 MIT指令: 12 字节/槽 = POS[23:0]+VEL[15:0]+T[15:0]+Kp[15:0]+Kd[15:0]+CANID
 * MIT 运算: Iq_ref = (Kp*(pos_des-pos_cur) + Kd*(vel_des-vel_cur) + Tff) / KT
 *
 * TODO: 本工程未实现完整 MIT 运算核。当前实现:
 *   - 仅保存位置目标和前馈扭矩到 controller
 *   - Kp/Kd 参数被丢弃，速度目标未使用
 *   - 使用 CYCLIC_SYNC_POSITION_MODE 承接，实际运行位置环
 *
 * 完整实现需要:
 *   1. 新增 MIT 控制模式枚举 (如 MIT_PD_MODE)
 *   2. 在 FOC 主循环中添加 MIT 运算分支
 *   3. 将 Kp*(pos_err) + Kd*(vel_err) + Tff 直接输出到 I_q_ref
 */
static void handle_mit_cmd(const uint8_t *data, uint32_t len) {
    int32_t off = find_slot(data, len, 12);
    if (off < 0) return;
    /* 解析 MIT 指令参数 */
    uint32_t p_raw = (uint32_t)data[off] | ((uint32_t)data[off + 1] << 8) |
                     ((uint32_t)data[off + 2] << 16);
    uint16_t v_raw = (uint16_t)data[off + 3] | ((uint16_t)data[off + 4] << 8);
    uint16_t t_raw = (uint16_t)data[off + 5] | ((uint16_t)data[off + 6] << 8);
    /* Kp/Kd 参数暂未使用 (TODO: 实现完整 MIT 运算) */
    // uint16_t kp_raw = (uint16_t)data[off + 7] | ((uint16_t)data[off + 8] << 8);
    // uint16_t kd_raw = (uint16_t)data[off + 9] | ((uint16_t)data[off + 10] << 8);

    float p_rad = uint_to_float(p_raw, g_can_wly_lim.pos_min, g_can_wly_lim.pos_max, 24);
    float t_nm = uint_to_float(t_raw, g_can_wly_lim.tq_min, g_can_wly_lim.tq_max, 16);
    (void)v_raw;  /* 速度目标暂未使用 */

    /* 临时方案: 将位置目标和前馈扭矩写入位置环 */
    controller_eyou.position_ref = pos_rad_to_int(p_rad);
    controller_eyou.I_q_ref = tq_nm_to_iq(t_nm);
    controller_eyou.controller_mode = CYCLIC_SYNC_POSITION_MODE;
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
    case CAN_WLY_OD_POS_MIN: g_can_wly_lim.pos_min = bytes_to_float_le(in); return 1;
    case CAN_WLY_OD_POS_MAX: g_can_wly_lim.pos_max = bytes_to_float_le(in); return 1;
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
            /* 保存到 Flash (temp1 低字节) */
            controller_eyou.FlashData.temp1 = (controller_eyou.FlashData.temp1 & 0xFFFFFF00) | new_id;
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
}

static void handle_ctrl_frame(const uint8_t *data, uint32_t len) {
    if (len < 8) return;
    for (int i = 0; i < 7; i++) {
        if (data[i] != 0xFF) return;
    }
    switch (data[7]) {
    case CAN_WLY_CTRL_ENABLE:
        controller_eyou.foc_run = 1;
        break;
    case CAN_WLY_CTRL_DISABLE:
        controller_eyou.foc_run = 0;
        controller_eyou.velocity_ref = 0;
        controller_eyou.I_q_ref = 0;
        break;
    case CAN_WLY_CTRL_SET_ZERO:
        /* 将当前机械位置设为零点 */
        controller_eyou.FlashData.mech_offest_out = controller_eyou.real_position_out +
                                                    controller_eyou.FlashData.mech_offest_out;
        controller_eyou.UserDataSaveFlag = 1;
        break;
    case CAN_WLY_CTRL_CLR_ERR:
        controller_eyou.ServoErrFlag.All_Flag = 0;
        break;
    default:
        return;
    }
    send_status_frame();
}

/* ========== 顶层 RX 分发 (覆盖 fdcan.c 的 weak 回调) ========== */
void fdcan_rx_user(uint32_t id, const uint8_t *data, uint32_t len) {
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
    /* 从 Flash 恢复节点 ID (存储在 FlashData.temp1 低字节) */
    uint8_t saved_id = (uint8_t)(controller_eyou.FlashData.temp1 & 0xFF);
    if (saved_id >= CAN_WLY_ID_MIN && saved_id <= CAN_WLY_ID_MAX) {
        s_node_id = saved_id;
    } else {
        s_node_id = CAN_WLY_ID_DEFAULT;
    }
}

uint8_t can_wly_get_node_id(void) { return s_node_id; }
void can_wly_set_node_id(uint8_t id) {
    if (id >= CAN_WLY_ID_MIN && id <= CAN_WLY_ID_MAX) s_node_id = id;
}

uint32_t can_wly_get_tx_fail_count(void) { return s_tx_fail_count; }

/* 1ms tick: 自动上报 + CAN 超时保护 */
void can_wly_tick_1ms(void) {
    if (s_auto_report) send_status_frame();

    if (s_can_timeout_enabled && s_can_timeout_cnt > 0) {
        if (--s_can_timeout_cnt == 0) {
            controller_eyou.ServoErrFlag.Bit.CommunicateErr = 1;
        }
    }
}
