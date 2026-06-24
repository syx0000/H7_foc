/**
 * @file    can_cybeast.c
 * @brief   守护兽 CAN Simple 协议从站实现
 *
 * Classic CAN, 1Mbps, 8B 标准帧
 * CAN_ID = (node_id << 5) | cmd_id
 *
 * Phase 1: MIT + Heartbeat + State + Estop + Clear
 * Phase 2: Pos/Vel/Torque + Encoder + Iq + Bus + Gains
 */

#include "can_protocol_sel.h"
#if (CAN_PROTOCOL_SEL == CAN_PROTO_CYBEAST)

#include "can_cybeast.h"
#include "fdcan.h"
#include "foc_api.h"
#include "can_wly.h"    /* can_wly_Nm_to_iA / can_wly_iA_to_Nm / Kt LUT */
#include "ifly_fault.h" /* getBoardTemp / getMotorTemp */
#include "tim.h"        /* htim1 */
#include "adc.h"        /* g_vdc */
#include <string.h>
#include <stdio.h>
#include <math.h>

/*============================================================================
 * 内部状态
 *============================================================================*/
static uint8_t  s_node_id = CB_NODE_ID_DEFAULT;
static volatile uint16_t s_can_timeout_cnt = 0;
static volatile uint16_t s_mit_timeout_cnt = 0;
static uint8_t  s_can_timeout_enabled = 0;
static uint8_t  s_hb_life = 0;
static uint16_t s_hb_tick = 0;

/* 外部引用 */
extern volatile uint8_t g_can_cali_request;
extern uint8_t g_can_timeout_force_disable;
/* g_vdc 已通过 adc.h 引入 */
extern uint8_t NPP;
extern uint32_t DEFAULT_MAX_SPEED;

/* MIT 量程 (可运行时通过 SDO 修改) */
static float s_mit_max_pos    = CB_MIT_MAX_POS;
static float s_mit_max_vel    = CB_MIT_MAX_VEL;
static float s_mit_max_kp     = CB_MIT_MAX_KP;
static float s_mit_max_kd     = CB_MIT_MAX_KD;
static float s_mit_max_torque = CB_MIT_MAX_TORQUE;

/*============================================================================
 * 工具函数
 *============================================================================*/

/* 量化整数 → 浮点 (守护兽 MIT 编码) */
static float uint_to_float_cb(uint32_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float max_val = (float)((1U << bits) - 1U);
    return ((float)x / max_val) * span + x_min;
}

/* 浮点 → 量化整数 */
static uint32_t float_to_uint_cb(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float max_val = (float)((1U << bits) - 1U);
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (uint32_t)((x - x_min) / span * max_val + 0.5f);
}

/* 构造 CAN ID */
static inline uint32_t cb_make_id(uint8_t cmd)
{
    return ((uint32_t)s_node_id << 5) | cmd;
}

/* 发送 8B 帧 */
static inline void cb_send(uint8_t cmd, const uint8_t *data, uint8_t len)
{
    uint8_t buf[8] = {0};
    if (data && len) {
        if (len > 8) len = 8;
        memcpy(buf, data, len);
    }
    fdcan_send(cb_make_id(cmd), buf, 8);
}

/*============================================================================
 * Heartbeat (0x01) - 周期性 TX
 *============================================================================*/

static uint32_t cybeast_build_axis_error(void)
{
    /* 映射 ServoErrFlag → 简化 32-bit 错误码 */
    return controller_eyou.ServoErrFlag.All_Flag;
}

static uint8_t cybeast_get_axis_state(void)
{
    if (controller_eyou.ServoErrFlag.All_Flag != 0)
        return CB_AXIS_STATE_IDLE;
    if (controller_eyou.foc_run == 2)
        return CB_AXIS_STATE_CLOSED_LOOP;
    return CB_AXIS_STATE_IDLE;
}

static uint8_t cybeast_build_flags(void)
{
    uint8_t flags = 0;
    uint32_t err = controller_eyou.ServoErrFlag.All_Flag;
    /* bit0: motor error (过流/过温/PWM相关) */
    if (err & 0x000F0000) flags |= 0x01;
    /* bit1: encoder error (编码器通信/数据) */
    if (err & 0x00000100) flags |= 0x02;
    /* bit2: controller error (超速/通信超时) */
    if (err & 0x00000003) flags |= 0x04;
    /* bit3: system error (过/欠压) */
    if (err & 0x0000000C) flags |= 0x08;
    /* bit7: trajectory_done (位置到达) */
    if (controller_eyou.ServoState.Bit.PositionArrivedFlag) flags |= 0x80;
    return flags;
}

static void heartbeat_send(void)
{
    uint32_t err = cybeast_build_axis_error();
    uint8_t state = cybeast_get_axis_state();
    uint8_t flags = cybeast_build_flags();

    /* 取驱动板温和电机温的较大值 (0.1°C 单位 → °C) */
    int16_t t_board = getBoardTemp();
    int16_t t_motor = getMotorTemp();
    int16_t t_max = (t_board > t_motor) ? t_board : t_motor;
    int8_t temp = (int8_t)((t_max / 10) > 127 ? 127 : (t_max / 10));

    uint8_t buf[8];
    buf[0] = (uint8_t)(err);
    buf[1] = (uint8_t)(err >> 8);
    buf[2] = (uint8_t)(err >> 16);
    buf[3] = (uint8_t)(err >> 24);
    buf[4] = state;
    buf[5] = flags;
    buf[6] = (uint8_t)temp;
    buf[7] = s_hb_life++;

    fdcan_send(cb_make_id(CB_CMD_HEARTBEAT), buf, 8);
}

/*============================================================================
 * MIT Control (0x08) - RX 解包 + TX 反馈
 *============================================================================*/

static void handle_mit_control(const uint8_t *d, uint32_t len)
{
    if (len < 8) return;

    /* 解包: Big-Endian bit-packed (守护兽协议格式) */
    uint16_t pos_raw = ((uint16_t)d[0] << 8) | d[1];
    uint16_t vel_raw = ((uint16_t)d[2] << 4) | (d[3] >> 4);
    uint16_t kp_raw  = ((uint16_t)(d[3] & 0x0F) << 8) | d[4];
    uint16_t kd_raw  = ((uint16_t)d[5] << 4) | (d[6] >> 4);
    uint16_t tq_raw  = ((uint16_t)(d[6] & 0x0F) << 8) | d[7];

    float p_des = uint_to_float_cb(pos_raw, -s_mit_max_pos, s_mit_max_pos, 16);
    float v_des = uint_to_float_cb(vel_raw, -s_mit_max_vel, s_mit_max_vel, 12);
    float kp    = uint_to_float_cb(kp_raw,  0.0f, s_mit_max_kp, 12);
    float kd    = uint_to_float_cb(kd_raw,  0.0f, s_mit_max_kd, 12);
    float t_nm  = uint_to_float_cb(tq_raw, -s_mit_max_torque, s_mit_max_torque, 12);

    /* N·m → A (经 Kt LUT) */
    float t_ff_A = can_wly_Nm_to_iA(t_nm);

    /* 读取反馈快照 (在关中断前读, 32-bit load 在 CM7 上原子;
     * 多变量之间可能有 1 拍不一致, 对 CAN 回帧非安全关键路径可接受) */
    int32_t fb_pos_raw = controller_eyou.real_position_out;
    int32_t fb_vel_raw = controller_eyou.dtheta_mech;
    int32_t fb_iq_raw  = controller_eyou.I_q;

    /* 写入控制器 (原子更新) */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    controller_eyou.mit_p_des = p_des;
    controller_eyou.mit_v_des = v_des;
    controller_eyou.mit_t_ff  = t_ff_A;
    controller_eyou.mit_kp    = kp;
    controller_eyou.mit_kd    = kd;
    controller_eyou.controller_mode = MIT_PD_MODE;
    s_mit_timeout_cnt = CB_MIT_TIMEOUT_MS;
    if (primask == 0U) __enable_irq();

    /* 发送 MIT 反馈帧 (6B 有效, 补 0 到 8B) */
    /* 格式: [node_id, pos_H, pos_L, vel_H|vel_L4, tq_H4|tq_L, 0, 0, 0] */
    /* 位置/速度/力矩均为输出轴侧 (与 MIT 输入一致) */
    float pos_fb = (float)fb_pos_raw / (180.0f * 1024.0f / (float)M_PI);  /* 1°/1024 → rad */
    float vel_fb = (float)fb_vel_raw / (1024.0f * 25.0f) * ((float)M_PI * 2.0f / 60.0f);  /* rpm×1024 电机端 → rad/s 输出端 */
    float tq_fb  = can_wly_iA_to_Nm((float)fb_iq_raw / 1024.0f);  /* 实际 Iq → Nm */

    uint16_t pos_fb_raw = float_to_uint_cb(pos_fb, -s_mit_max_pos, s_mit_max_pos, 16);
    uint16_t vel_fb_raw = float_to_uint_cb(vel_fb, -s_mit_max_vel, s_mit_max_vel, 12);
    uint16_t tq_fb_raw  = float_to_uint_cb(tq_fb, -s_mit_max_torque, s_mit_max_torque, 12);

    uint8_t fb[8] = {0};
    fb[0] = s_node_id;
    fb[1] = (uint8_t)(pos_fb_raw >> 8);
    fb[2] = (uint8_t)(pos_fb_raw);
    fb[3] = (uint8_t)(vel_fb_raw >> 4);
    fb[4] = (uint8_t)(((vel_fb_raw & 0x0F) << 4) | (tq_fb_raw >> 8));
    fb[5] = (uint8_t)(tq_fb_raw);

    fdcan_send(cb_make_id(CB_CMD_MIT_CONTROL), fb, 8);
}

/*============================================================================
 * Set Axis State (0x07)
 *============================================================================*/

static void handle_set_axis_state(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    uint32_t state = d[0] | ((uint32_t)d[1] << 8) |
                     ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);

    switch (state) {
    case CB_AXIS_STATE_IDLE:
        controller_eyou.foc_run = 0;
        controller_eyou.I_q_ref = 0;
        controller_eyou.I_d_ref = 0;
        TIM1->CCER &= ~0x0555u;
        break;

    case CB_AXIS_STATE_CLOSED_LOOP:
        controller_eyou.ServoErrFlag.All_Flag = 0;
        controller_eyou.foc_run = 2;
        TIM1->CCER |= 0x0555u;
        __HAL_TIM_MOE_ENABLE(&htim1);
        break;

    case CB_AXIS_STATE_FULL_CALIB:
        /* 全辨识: Rs/Ld/Lq + 电角度 (非阻塞, main loop 里跑 identifyMotorParamsCached) */
        /* 先失能再辨识 */
        controller_eyou.foc_run = 0;
        controller_eyou.FlashData.MotorParamFlag = 0;  /* 强制重新辨识 */
        g_can_cali_request = 1;
        break;

    case CB_AXIS_STATE_MOTOR_CALIB:
        /* 电机参数辨识 (Rs/Ld/Lq), 通过 bwtest3 触发 */
        controller_eyou.foc_run = 0;
        controller_eyou.FlashData.MotorParamFlag = 0;
        g_can_cali_request = 1;
        break;

    case CB_AXIS_STATE_ENCODER_CALIB:
        /* 电角度辨识 */
        g_can_cali_request = 1;
        break;

    default:
        break;
    }
}

/*============================================================================
 * Get Error (0x03)
 *============================================================================*/

static void handle_get_error(const uint8_t *d, uint32_t len)
{
    /* 输入: byte0 = Error_Type (0=motor, 1=encoder, 3=controller, 4=system) */
    uint8_t err_type = (len >= 1) ? d[0] : 0;
    uint8_t buf[8] = {0};
    uint32_t err_val = controller_eyou.ServoErrFlag.All_Flag;

    /* 当前硬件统一用一个 32-bit flag, 按类型返回对应位段 */
    switch (err_type) {
    case 0: /* motor error - 返回 8 字节 (协议要求 uint64) */
        memcpy(buf, &err_val, 4);
        break;
    case 1: /* encoder error */
        buf[0] = 0; /* 暂无独立编码器错误码 */
        break;
    case 3: /* controller error */
        buf[0] = (err_val & 0x01) ? 1 : 0;  /* CommunicateErr → overspeed bit */
        break;
    case 4: /* system error */
        memcpy(buf, &err_val, 4);
        break;
    default:
        break;
    }
    cb_send(CB_CMD_GET_ERROR, buf, 8);
}

/*============================================================================
 * Set Axis Node ID (0x06)
 *============================================================================*/

static void handle_set_node_id(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    uint32_t new_id = d[0] | ((uint32_t)d[1] << 8) |
                      ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    if (new_id >= CB_NODE_ID_MIN && new_id <= CB_NODE_ID_MAX) {
        s_node_id = (uint8_t)new_id;
        /* 持久化到 Flash (temp4 低字节) */
        controller_eyou.FlashData.temp4 =
            (controller_eyou.FlashData.temp4 & 0xFFFFFF00u) | s_node_id;
    }
}

/*============================================================================
 * Set Traj Vel Limit (0x11)
 *============================================================================*/

static void handle_set_traj_vel_limit(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    float traj_vel;
    memcpy(&traj_vel, d, 4);  /* rev/s, 输出端 */
    /* 映射到梯形规划最大速度: v_max 内部单位 = rpm × POS_TRAPEZOID_VMAX_SCALE */
    if (traj_vel > 0.0f) {
        float rpm = traj_vel * 60.0f;
        controller_eyou.SmoothPosRef.v_max = rpm * (1024.0f * 25.0f / 60.0f);
    }
}

/*============================================================================
 * Set Traj Accel Limits (0x12)
 *============================================================================*/

static void handle_set_traj_accel(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    float accel;
    memcpy(&accel, d, 4);  /* rev/s^2, 输出端 */
    /* 映射到梯形规划加速度: a_max 内部单位 = rpm/s × POS_TRAPEZOID_AMAX_SCALE */
    if (accel > 0.0f) {
        float rpms = accel * 60.0f;
        controller_eyou.SmoothPosRef.a_max = rpms * (1024.0f * 25.0f / 60.0f);
    }

    /* decel: byte4-7 (可选) */
    if (len >= 8) {
        /* 当前系统不区分加减速, 忽略 decel */
    }
}

/*============================================================================
 * Set Traj Inertia (0x13)
 *============================================================================*/

static void handle_set_traj_inertia(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    float inertia;
    memcpy(&inertia, d, 4);  /* Nm/(rev/s^2) */
    /* 暂存, 当前控制律不使用外部惯量参数 */
    (void)inertia;
}

/*============================================================================
 * Get Thermistor Temperature (0x15)
 *============================================================================*/

static void handle_get_temperature(void)
{
    /* 协议: byte0-3 = motor_temp (float °C), byte4-7 = fet_temp (float °C) */
    float t_motor = (float)getMotorTemp() / 10.0f;
    float t_board = (float)getBoardTemp() / 10.0f;

    uint8_t buf[8] = {0};
    memcpy(&buf[0], &t_motor, 4);
    memcpy(&buf[4], &t_board, 4);
    cb_send(0x15, buf, 8);
}

/*============================================================================
 * Get Encoder Count (0x0A)
 *============================================================================*/

static void handle_get_encoder_count(void)
{
    /* shadow_count: 多圈计数 (转子侧) */
    int32_t shadow = (int32_t)((float)controller_eyou.real_position_out *
                               CAN_WLY_GR / (360.0f * 1024.0f) * 16384.0f);
    /* count_in_cpr: 单圈计数 */
    int32_t cpr = (int32_t)(controller_eyou.now_mechposition & 0x00FFFFFF);

    uint8_t buf[8] = {0};
    memcpy(&buf[0], &shadow, 4);
    memcpy(&buf[4], &cpr, 4);
    cb_send(CB_CMD_GET_ENCODER_COUNT, buf, 8);
}

/*============================================================================
 * Set Move Incremental (0x19)
 *============================================================================*/

static void handle_set_move_incremental(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    float displacement;
    memcpy(&displacement, d, 4);  /* rev, 转子侧增量 */

    /* 转子侧 rev → 输出端 1°/1024 */
    float output_rev = displacement / CAN_WLY_GR;
    int32_t delta = (int32_t)(output_rev * 360.0f * 1024.0f);
    controller_eyou.position_ref += delta;
}

/*============================================================================
 * Get Powers (0x1D)
 *============================================================================*/

static void handle_get_powers(void)
{
    /* Electrical_Power = Vbus × Ibus (近似: Vbus × Iq × cos(phi) ≈ Vbus × Iq) */
    float iq_A = (float)controller_eyou.I_q / 1024.0f;
    float elec_power = g_vdc * iq_A;

    /* Mechanical_Power = Torque × omega */
    float tq_nm = can_wly_iA_to_Nm(iq_A);
    float omega_rads = (float)controller_eyou.dtheta_mech / (1024.0f * 25.0f) *
                       (2.0f * (float)M_PI / 60.0f);  /* 输出端 rad/s */
    float mech_power = tq_nm * omega_rads;

    uint8_t buf[8] = {0};
    memcpy(&buf[0], &elec_power, 4);
    memcpy(&buf[4], &mech_power, 4);
    cb_send(0x1D, buf, 8);
}

/*============================================================================
 * RxSdo / TxSdo (0x04 / 0x05) - 通用端点访问
 *============================================================================*/

/* 简化 endpoint 表: 映射常用参数 */
typedef struct {
    uint16_t ep_id;
    uint8_t  type;   /* 'f'=float32, 'u'=uint32, 'i'=int32, 'b'=uint8 */
    void    *ptr;    /* 指向变量的指针 */
    uint8_t  rw;     /* 0=RO, 1=RW */
} cb_endpoint_t;

static float s_gear_ratio = CAN_WLY_GR;

static cb_endpoint_t s_endpoints[] = {
    /* 常用电机参数 */
    {0x0001, 'f', NULL, 0},                                        /* vbus_voltage (动态) */
    {0x0002, 'f', &s_gear_ratio, 1},                               /* gear_ratio */
    {0x0010, 'i', NULL, 0},                                        /* pole_pairs (动态) */
    {0x0020, 'f', NULL, 0},                                        /* torque_constant (动态) */
    /* MIT 量程 */
    {0x0100, 'f', &s_mit_max_pos, 1},
    {0x0101, 'f', &s_mit_max_vel, 1},
    {0x0102, 'f', &s_mit_max_kp, 1},
    {0x0103, 'f', &s_mit_max_kd, 1},
    {0x0104, 'f', &s_mit_max_torque, 1},
    /* node_id */
    {0x0200, 'b', &s_node_id, 1},
};
#define CB_EP_COUNT (sizeof(s_endpoints) / sizeof(s_endpoints[0]))

static void handle_rxsdo(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    uint8_t opcode = d[0];           /* 0=read, 1=write */
    uint16_t ep_id = (uint16_t)d[1] | ((uint16_t)d[2] << 8);

    /* 找 endpoint */
    const cb_endpoint_t *ep = NULL;
    for (uint32_t i = 0; i < CB_EP_COUNT; i++) {
        if (s_endpoints[i].ep_id == ep_id) {
            ep = &s_endpoints[i];
            break;
        }
    }

    uint8_t resp[8] = {0};
    resp[0] = 0;       /* opcode = 0 (response) */
    resp[1] = d[1];    /* echo endpoint_id */
    resp[2] = d[2];

    if (!ep) {
        /* endpoint 不存在 → 返回错误 */
        resp[0] = 0x80;
        cb_send(CB_CMD_TXSDO, resp, 8);
        return;
    }

    if (opcode == 0) {
        /* READ */
        /* 动态端点特殊处理 */
        if (ep_id == 0x0001) {
            float v = g_vdc;
            memcpy(&resp[4], &v, 4);
        } else if (ep_id == 0x0010) {
            uint32_t pp = NPP;
            memcpy(&resp[4], &pp, 4);
        } else if (ep_id == 0x0020) {
            float kt = g_can_wly_kt_out;
            memcpy(&resp[4], &kt, 4);
        } else if (ep->ptr) {
            switch (ep->type) {
            case 'f': memcpy(&resp[4], ep->ptr, 4); break;
            case 'u': memcpy(&resp[4], ep->ptr, 4); break;
            case 'i': memcpy(&resp[4], ep->ptr, 4); break;
            case 'b': resp[4] = *(uint8_t*)ep->ptr; break;
            }
        }
        cb_send(CB_CMD_TXSDO, resp, 8);

    } else if (opcode == 1) {
        /* WRITE */
        if (!ep->rw || !ep->ptr) {
            resp[0] = 0x80;  /* read-only error */
            cb_send(CB_CMD_TXSDO, resp, 8);
            return;
        }
        if (len >= 8) {
            switch (ep->type) {
            case 'f': memcpy(ep->ptr, &d[4], 4); break;
            case 'u': memcpy(ep->ptr, &d[4], 4); break;
            case 'i': memcpy(ep->ptr, &d[4], 4); break;
            case 'b': *(uint8_t*)ep->ptr = d[4]; break;
            }
        }
        /* ACK */
        cb_send(CB_CMD_TXSDO, resp, 8);
    }
}

/*============================================================================
 * Estop (0x02)
 *============================================================================*/

static void handle_estop(void)
{
    controller_eyou.foc_run = 0;
    controller_eyou.I_q_ref = 0;
    controller_eyou.I_d_ref = 0;
    controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
    TIM1->CCER &= ~0x0555u;
    controller_eyou.ServoErrFlag.Bit.CommunicateErr = 1;
}

/*============================================================================
 * Clear Errors (0x18)
 *============================================================================*/

static void handle_clear_errors(void)
{
    controller_eyou.ServoErrFlag.All_Flag = 0;
}

/*============================================================================
 * Set Controller Mode (0x0B)
 *============================================================================*/

static void handle_set_controller_mode(const uint8_t *d, uint32_t len)
{
    if (len < 8) return;
    uint32_t ctrl_mode  = d[0] | ((uint32_t)d[1] << 8) |
                          ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    uint32_t input_mode = d[4] | ((uint32_t)d[5] << 8) |
                          ((uint32_t)d[6] << 16) | ((uint32_t)d[7] << 24);

    switch (ctrl_mode) {
    case CB_CTRL_MODE_TORQUE:
        controller_eyou.controller_mode = PROFILE_TORQUE_MODE;
        break;
    case CB_CTRL_MODE_VELOCITY:
        controller_eyou.controller_mode = CYCLIC_SYNC_VELOCITY_MODE;
        break;
    case CB_CTRL_MODE_POSITION:
        /* input_mode=5(梯形) → PROFILE_POSITION, 其他 → CSP(直通) */
        if (input_mode == CB_INPUT_MODE_TRAP_TRAJ) {
            controller_eyou.controller_mode = PROFILE_POSITION_MODE;
        } else {
            controller_eyou.controller_mode = CYCLIC_SYNC_POSITION_MODE;
        }
        break;
    default:
        break;
    }
}

/*============================================================================
 * Set Input Torque (0x0E)
 *============================================================================*/

static void handle_set_input_torque(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    float torque_nm;
    memcpy(&torque_nm, d, 4);  /* IEEE 754 LE */

    float i_A = can_wly_Nm_to_iA(torque_nm);
    int32_t iq_q10 = (int32_t)(i_A * 1024.0f);

    controller_eyou.I_q_ref = iq_q10;
}

/*============================================================================
 * Set Input Velocity (0x0D)
 *============================================================================*/

static void handle_set_input_vel(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    float vel_revs;
    memcpy(&vel_revs, d, 4);  /* rev/s, 输出端 */

    /* rev/s → rpm×1024×25 (内部单位) */
    int32_t vel_internal = (int32_t)(vel_revs * 60.0f * 1024.0f * 25.0f);
    controller_eyou.velocity_ref = vel_internal;

    /* Torque_FF: float32 @ byte4-7, 单位 Nm */
    if (len >= 8) {
        float tq_ff_nm;
        memcpy(&tq_ff_nm, &d[4], 4);
        float i_ff = can_wly_Nm_to_iA(tq_ff_nm);
        /* 速度模式力矩前馈 → I_q_ref 偏置 */
        (void)i_ff;  /* 预留: 当前速度环不支持力矩前馈注入 */
    }
}

/*============================================================================
 * Set Input Position (0x0C)
 *============================================================================*/

static void handle_set_input_pos(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    float pos_rev;
    memcpy(&pos_rev, d, 4);  /* rev, 输出端 */

    /* rev → 1°/1024 (内部单位) */
    int32_t pos_internal = (int32_t)(pos_rev * 360.0f * 1024.0f);
    controller_eyou.position_ref = pos_internal;

    /* Vel_FF: int16 @ byte4-5, 单位 0.001 rev/s */
    if (len >= 6) {
        int16_t vel_ff_raw = (int16_t)((uint16_t)d[4] | ((uint16_t)d[5] << 8));
        float vel_ff_revs = (float)vel_ff_raw * 0.001f;
        /* rev/s 输出端 → rpm×1024×25 内部 */
        controller_eyou.velocity_ref = (int32_t)(vel_ff_revs * 60.0f * 1024.0f * 25.0f);
    }

    /* Torque_FF: int16 @ byte6-7, 单位 0.001 Nm */
    if (len >= 8) {
        int16_t tq_ff_raw = (int16_t)((uint16_t)d[6] | ((uint16_t)d[7] << 8));
        float tq_ff_nm = (float)tq_ff_raw * 0.001f;
        float i_ff = can_wly_Nm_to_iA(tq_ff_nm);
        /* 力矩前馈叠加到 I_q_ref (如果需要) */
        (void)i_ff;  /* CSP 模式下位置环自带速度/力矩前馈，此处预留 */
    }
}

/*============================================================================
 * Get Encoder Estimates (0x09)
 *============================================================================*/

static void handle_get_encoder(void)
{
    /* 协议要求转子侧: pos_estimate (rev), vel_estimate (rev/s) */
    /* real_position_out: 输出端 1°/1024 → 转子侧 rev = (pos / 360 / 1024) * gear_ratio */
    float pos_rev = (float)controller_eyou.real_position_out / (360.0f * 1024.0f) * CAN_WLY_GR;
    /* dtheta_mech: 电机端 rpm×1024 → 转子侧 rev/s = rpm/60 */
    float vel_revs = (float)controller_eyou.dtheta_mech / (1024.0f * 60.0f);

    uint8_t buf[8] = {0};
    memcpy(&buf[0], &pos_rev, 4);
    memcpy(&buf[4], &vel_revs, 4);
    cb_send(CB_CMD_GET_ENCODER, buf, 8);
}

/*============================================================================
 * Get Iq (0x14)
 *============================================================================*/

static void handle_get_iq(void)
{
    float iq_set = (float)controller_eyou.I_q_ref / 1024.0f;
    float iq_meas = (float)controller_eyou.I_q / 1024.0f;

    uint8_t buf[8] = {0};
    memcpy(&buf[0], &iq_set, 4);
    memcpy(&buf[4], &iq_meas, 4);
    cb_send(CB_CMD_GET_IQ, buf, 8);
}

/*============================================================================
 * Get Bus Voltage / Current (0x17)
 *============================================================================*/

static void handle_get_bus_voltage(void)
{
    float vbus = g_vdc;
    float ibus = 0.0f;  /* 暂无母线电流测量 */

    uint8_t buf[8] = {0};
    memcpy(&buf[0], &vbus, 4);
    memcpy(&buf[4], &ibus, 4);
    cb_send(CB_CMD_GET_BUS_VOLTAGE, buf, 8);
}

/*============================================================================
 * Set Pos Gain (0x1A)
 *============================================================================*/

static void handle_set_pos_gain(const uint8_t *d, uint32_t len)
{
    if (len < 4) return;
    float pos_gain;
    memcpy(&pos_gain, d, 4);
    /* 守护兽单位: (rev/s)/rev, 内部: IncPID_Position.P (Q0 整数)
     * 简化映射: 直接写入 (后续可加精确换算) */
    controller_eyou.IncPID_Position.P = (int32_t)(pos_gain * 100.0f);
}

/*============================================================================
 * Set Vel Gains (0x1B)
 *============================================================================*/

static void handle_set_vel_gains(const uint8_t *d, uint32_t len)
{
    if (len < 8) return;
    float vel_gain, vel_integrator;
    memcpy(&vel_gain, &d[0], 4);
    memcpy(&vel_integrator, &d[4], 4);

    controller_eyou.IncPID_Speed.P = (int32_t)(vel_gain * 100.0f);
    controller_eyou.IncPID_Speed.I = (int32_t)(vel_integrator * 100.0f);
}

/*============================================================================
 * Set Limits (0x0F)
 *============================================================================*/

static void handle_set_limits(const uint8_t *d, uint32_t len)
{
    if (len < 8) return;
    float vel_limit_revs, current_limit_A;
    memcpy(&vel_limit_revs, &d[0], 4);
    memcpy(&current_limit_A, &d[4], 4);

    /* vel_limit: rev/s 输出端 → rpm×1024×25 电机端 */
    if (vel_limit_revs > 0.0f) {
        int32_t new_max = (int32_t)(vel_limit_revs * 60.0f * 1024.0f * 25.0f);
        DEFAULT_MAX_SPEED = (uint32_t)new_max;
        /* 速度环指令限幅 (set_velocity_ref_loop 用此值钳位 velocity_ref) */
        controller_eyou.FlashData.MaxSpeed = DEFAULT_MAX_SPEED;
        /* 位置环总输出限幅 = MaxSpeed (PID+FF 合计), 防止速度环看到大幅截断的方波 */
        controller_eyou.FlashData.Pid_PositionLimit = new_max;
        /* 位置环 PID OutputMax 略大于 MaxSpeed (给 FF 留 20% 余量), 保持 PID 线性工作 */
        controller_eyou.IncPID_Position.OutputMax = new_max + new_max / 5;
        /* 速度环斜坡: 更新加速度 + 钳位内部状态 */
        controller_eyou.SpeedSmooth.MaxVelAccEveryPrd = DEFAULT_MAX_SPEED / MIN_ACC_TIME;
        if (controller_eyou.SpeedSmooth.NowVelocityRef >  new_max)
            controller_eyou.SpeedSmooth.NowVelocityRef =  new_max;
        if (controller_eyou.SpeedSmooth.NowVelocityRef < -new_max)
            controller_eyou.SpeedSmooth.NowVelocityRef = -new_max;
        controller_eyou.SpeedSmooth.OldVelocityRef = controller_eyou.SpeedSmooth.NowVelocityRef;
        /* 梯形规划: 更新巡航速度 + 钳位当前规划速度 (留 10% 给速度环跟踪) */
        float v_max_new = vel_limit_revs * 60.0f * 2.4576f * 0.9f; /* POS_TRAPEZOID_VMAX_SCALE */
        controller_eyou.SmoothPosRef.v_max = v_max_new;
        if (controller_eyou.SmoothPosRef.cur_v >  v_max_new)
            controller_eyou.SmoothPosRef.cur_v =  v_max_new;
        if (controller_eyou.SmoothPosRef.cur_v < -v_max_new)
            controller_eyou.SmoothPosRef.cur_v = -v_max_new;
    }

    /* current_limit: A → Q10 */
    if (current_limit_A > 0.0f) {
        int32_t lim_q10 = (int32_t)(current_limit_A * 1024.0f);
        controller_eyou.IncPID_Speed.OutputMax = lim_q10;
        /* IncPID 结构体无 OutputMin，对称限幅由 PidRun 内部处理 */
    }
}

/*============================================================================
 * Get Torques (0x1C)
 *============================================================================*/

static void handle_get_torques(void)
{
    float tq_set = can_wly_iA_to_Nm((float)controller_eyou.I_q_ref / 1024.0f);
    float tq_meas = can_wly_iA_to_Nm((float)controller_eyou.I_q / 1024.0f);

    uint8_t buf[8] = {0};
    memcpy(&buf[0], &tq_set, 4);
    memcpy(&buf[4], &tq_meas, 4);
    cb_send(CB_CMD_GET_TORQUES, buf, 8);
}

/*============================================================================
 * Reboot (0x16) / Disable_Can (0x1E) - 延迟到 main loop 执行
 * (fdcan_rx_user 在 ISR 中调用, HAL_Delay 会死锁)
 *============================================================================*/

static volatile uint8_t s_pending_reboot = 0;
static volatile uint8_t s_pending_save_reset = 0;
static volatile uint8_t s_pending_disable_can = 0;

static void handle_save_config(void)
{
    s_pending_save_reset = 1;
}

static void handle_reboot(void)
{
    s_pending_reboot = 1;
}

static void handle_disable_can(void)
{
    s_pending_disable_can = 1;
}

/*============================================================================
 * RX Dispatch
 *============================================================================*/

void can_cybeast_rx_dispatch(uint32_t id, const uint8_t *data, uint32_t len)
{
    uint8_t rx_node = (uint8_t)(id >> 5);
    uint8_t cmd     = (uint8_t)(id & 0x1F);

    /* 只响应本节点 ID */
    if (rx_node != s_node_id) return;

    /* 喂看门狗 */
    s_can_timeout_cnt = CB_CAN_TIMEOUT_MS;
    if (!s_can_timeout_enabled) s_can_timeout_enabled = 1;

    switch (cmd) {
    case CB_CMD_HEARTBEAT:
        /* 主机不应发 heartbeat 给从机, 忽略 */
        break;

    case CB_CMD_ESTOP:
        handle_estop();
        break;

    case CB_CMD_GET_ERROR:
        handle_get_error(data, len);
        break;

    case CB_CMD_RXSDO:
        handle_rxsdo(data, len);
        break;

    case CB_CMD_SET_NODE_ID:
        handle_set_node_id(data, len);
        break;

    case CB_CMD_SET_AXIS_STATE:
        handle_set_axis_state(data, len);
        break;

    case CB_CMD_MIT_CONTROL:
        handle_mit_control(data, len);
        break;

    case CB_CMD_GET_ENCODER:
        handle_get_encoder();
        break;

    case CB_CMD_GET_ENCODER_COUNT:
        handle_get_encoder_count();
        break;

    case CB_CMD_SET_CONTROLLER_MODE:
        handle_set_controller_mode(data, len);
        break;

    case CB_CMD_SET_INPUT_POS:
        handle_set_input_pos(data, len);
        break;

    case CB_CMD_SET_INPUT_VEL:
        handle_set_input_vel(data, len);
        break;

    case CB_CMD_SET_INPUT_TORQUE:
        handle_set_input_torque(data, len);
        break;

    case CB_CMD_SET_LIMITS:
        handle_set_limits(data, len);
        break;

    case CB_CMD_SET_TRAJ_VEL_LIMIT:
        handle_set_traj_vel_limit(data, len);
        break;

    case CB_CMD_SET_TRAJ_ACCEL:
        handle_set_traj_accel(data, len);
        break;

    case CB_CMD_SET_TRAJ_INERTIA:
        handle_set_traj_inertia(data, len);
        break;

    case 0x15: /* Get_Thermistor_Temperature */
        handle_get_temperature();
        break;

    case CB_CMD_GET_IQ:
        handle_get_iq();
        break;

    case CB_CMD_GET_BUS_VOLTAGE:
        handle_get_bus_voltage();
        break;

    case CB_CMD_GET_TORQUES:
        handle_get_torques();
        break;

    case CB_CMD_SET_POS_GAIN:
        handle_set_pos_gain(data, len);
        break;

    case CB_CMD_SET_VEL_GAINS:
        handle_set_vel_gains(data, len);
        break;

    case CB_CMD_CLEAR_ERRORS:
        handle_clear_errors();
        break;

    case 0x19: /* Set_Move_Incremental */
        handle_set_move_incremental(data, len);
        break;

    case 0x1D: /* Get_Powers */
        handle_get_powers();
        break;

    case 0x1E: /* Disable_Can */
        handle_disable_can();
        break;

    case CB_CMD_SAVE_CONFIG:
        handle_save_config();
        break;

    case CB_CMD_REBOOT:
        handle_reboot();
        break;

    default:
        break;
    }
}

/*============================================================================
 * Init
 *============================================================================*/

void can_cybeast_init(void)
{
    /* 从 Flash 恢复 node_id (复用 WLY 的 temp4 低字节) */
    uint8_t saved_id = (uint8_t)(controller_eyou.FlashData.temp4 & 0xFF);
    if (saved_id >= CB_NODE_ID_MIN && saved_id <= CB_NODE_ID_MAX) {
        s_node_id = saved_id;
    } else {
        s_node_id = CB_NODE_ID_DEFAULT;
    }

    printf("CyberBeast CAN Simple init, node_id=%d\r\n", s_node_id);
}

/*============================================================================
 * 1ms Tick (超时 + Heartbeat)
 *============================================================================*/

void can_cybeast_tick_1ms(void)
{
    /* CAN 通信超时 */
    if (s_can_timeout_enabled && !g_can_timeout_force_disable && s_can_timeout_cnt > 0) {
        if (--s_can_timeout_cnt == 0) {
            controller_eyou.ServoErrFlag.Bit.CommunicateErr = 1;
        }
    }

    /* MIT 超时 */
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

    /* Heartbeat */
    if (++s_hb_tick >= CB_HEARTBEAT_MS) {
        s_hb_tick = 0;
        heartbeat_send();
    }
}

/*============================================================================
 * Poll (预留)
 *============================================================================*/

void can_cybeast_poll(void)
{
    extern void WriteDataToFlash(void);
    extern HAL_StatusTypeDef Flash_EraseSector(void);

    /* Save + Reset (延迟自 ISR) */
    if (s_pending_save_reset) {
        s_pending_save_reset = 0;
        if (Flash_EraseSector() == HAL_OK) {
            WriteDataToFlash();
        }
        HAL_Delay(100);
        NVIC_SystemReset();
    }

    /* Reboot (延迟自 ISR) */
    if (s_pending_reboot) {
        s_pending_reboot = 0;
        HAL_Delay(50);
        NVIC_SystemReset();
    }

    /* Disable CAN (延迟自 ISR) */
    if (s_pending_disable_can) {
        s_pending_disable_can = 0;
        HAL_FDCAN_Stop(&hfdcan1);
        HAL_Delay(50);
        NVIC_SystemReset();
    }
}

/*============================================================================
 * Getter/Setter
 *============================================================================*/

uint8_t can_cybeast_get_node_id(void)
{
    return s_node_id;
}

void can_cybeast_set_node_id(uint8_t id)
{
    if (id >= CB_NODE_ID_MIN && id <= CB_NODE_ID_MAX) {
        s_node_id = id;
    }
}

#endif /* CAN_PROTOCOL_SEL == CAN_PROTO_CYBEAST */
