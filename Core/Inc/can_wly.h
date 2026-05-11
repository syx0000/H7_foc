/**
 * @file    can_wly.h
 * @brief   万里扬FDCAN通信协议V1.7 - 从站实现
 * @details 参考:
 *   - 万里扬FDCAN通信协议V1.7-1.pdf (工程根目录)
 *   - motor_h7_0426/UserSrc/Src/can_rv.c (H7参考实现)
 *
 * 帧ID布局: [10:7]=category | [6:0]=CANID (节点地址, 默认 1, 范围 1..127)
 *   0x080        : 广播状态查询 -> 从站回 0x100+ID (12 字节状态帧)
 *   0x100+ID     : 从站主动/回复状态帧 (POS[24] VEL[16] T[16] ERR1[16] ERR2[8] WARN[8] STA[8])
 *   0x200        : 速度指令 (每从站 3 字节: V_des[16] + CANID[8])
 *   0x300        : 转矩指令 (每从站 3 字节: T_des[16] + CANID[8])
 *   0x400        : 位置指令 (每从站 6 字节: POS[24] + V_des[16] + CANID[8])
 *   0x500        : MIT 指令 (每从站 12 字节: POS+VEL+T+Kp+Kd+CANID)
 *   0x600+ID     : SDO 读(0x40) / 写(0x23)
 *   0x580+ID     : SDO 应答 (0x60 成功 / 0x80 错误)
 *   0x700+ID     : 控制帧: 0xFA 使能 / 0xFB 失能 / 0xFC 置零 / 0xFD 清错
 */
#ifndef __CAN_WLY_H__
#define __CAN_WLY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* 节点地址范围 */
#define CAN_WLY_ID_MIN          1
#define CAN_WLY_ID_MAX          127
#define CAN_WLY_ID_DEFAULT      1

/* 帧 ID 类别基址 */
#define CAN_WLY_ID_QUERY_BCAST  0x080
#define CAN_WLY_ID_STATUS_BASE  0x100
#define CAN_WLY_ID_SPEED        0x200
#define CAN_WLY_ID_TORQUE       0x300
#define CAN_WLY_ID_POSITION     0x400
#define CAN_WLY_ID_MIT          0x500
#define CAN_WLY_ID_SDO_RSP_BASE 0x580
#define CAN_WLY_ID_SDO_REQ_BASE 0x600
#define CAN_WLY_ID_CTRL_BASE    0x700

/* 控制帧尾字节 (D[7]) */
#define CAN_WLY_CTRL_ENABLE     0xFA
#define CAN_WLY_CTRL_DISABLE    0xFB
#define CAN_WLY_CTRL_SET_ZERO   0xFC
#define CAN_WLY_CTRL_CLR_ERR    0xFD

/* SDO 命令字节 */
#define CAN_WLY_SDO_READ_REQ    0x40
#define CAN_WLY_SDO_WRITE_REQ   0x23
#define CAN_WLY_SDO_ACK         0x60
#define CAN_WLY_SDO_ERR         0x80

/* 对象字典索引 (参考协议 1.9 节) */
#define CAN_WLY_OD_POS_MIN      0x2000  /* float32, rad */
#define CAN_WLY_OD_POS_MAX      0x2001  /* float32, rad */
#define CAN_WLY_OD_SPD_MIN      0x2002  /* float32, rad/s */
#define CAN_WLY_OD_SPD_MAX      0x2003  /* float32, rad/s */
#define CAN_WLY_OD_TQ_MIN       0x2004  /* float32, N·m */
#define CAN_WLY_OD_TQ_MAX       0x2005  /* float32, N·m */
#define CAN_WLY_OD_KP_MIN       0x2006  /* float32 */
#define CAN_WLY_OD_KP_MAX       0x2007
#define CAN_WLY_OD_KD_MIN       0x2008
#define CAN_WLY_OD_KD_MAX       0x2009
#define CAN_WLY_OD_SYNC_CYCLE   0x200A  /* uint16, Hz */
#define CAN_WLY_OD_DRI_POS_KP   0x200B
#define CAN_WLY_OD_DRI_SPD_KP   0x200C
#define CAN_WLY_OD_DRI_SPD_KI   0x200D
#define CAN_WLY_OD_NODE_ID      0x2F00  /* 节点地址 (自定义, 参考 motor_h7) */
#define CAN_WLY_OD_AUTO_REPORT  0x2F05  /* 主动上报模式: 0=关闭, 2=开启 */

/* 控制模式 (匹配 foc_controller.h 中的 CIA402 模式常量) */

/* 初始化: 注册协议处理句柄。main 中 MX_FDCAN1_Init 之后调用 */
void can_wly_init(void);

/* 1kHz 轮询, 建议放在 SysTick 或 1ms 定时器里。处理: 状态主动上报 */
void can_wly_tick_1ms(void);

/* 获取/设置节点地址 (运行时可由 0x600/SDO 写 0x2F00 改) */
uint8_t can_wly_get_node_id(void);
void    can_wly_set_node_id(uint8_t id);

/* 获取发送失败计数 (调试用) */
uint32_t can_wly_get_tx_fail_count(void);

/* 对外暴露的边界参数 (单位: rad / rad·s / N·m) */
typedef struct {
    float pos_min, pos_max;   /* rad */
    float spd_min, spd_max;   /* rad/s */
    float tq_min,  tq_max;    /* N·m */
    float kp_min,  kp_max;
    float kd_min,  kd_max;
} can_wly_limits_t;
extern can_wly_limits_t g_can_wly_lim;

/* 扭矩常数 KT (N·m / A, q轴电流 -> 输出端扭矩, 含减速比)
 *   motor_h7: KT_OUT = Kt * GR. 这里暂以 1.0 估算, 标定后通过 SDO 修改 */
extern float g_can_wly_kt_out;

/* 减速比 (用于 rpm/rad-s 换算. 对齐 motor_h7: GR=25 对应 CLAUDE.md) */
#define CAN_WLY_GR          25.0f

#ifdef __cplusplus
}
#endif

#endif /* __CAN_WLY_H__ */
