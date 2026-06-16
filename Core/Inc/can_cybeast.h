/**
 * @file    can_cybeast.h
 * @brief   守护兽 CAN Simple 协议 - 从站实现 (ODrive 兼容)
 *
 * 帧格式: 标准帧 11-bit ID, 8 字节数据, Classic CAN 1Mbps
 * CAN_ID = (node_id << 5) | cmd_id
 *
 * 参考: 守护兽驱动协议手册 - CAN Simple 协议章节
 */
#ifndef __CAN_CYBEAST_H__
#define __CAN_CYBEAST_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*============================================================================
 * CMD ID 定义
 *============================================================================*/
#define CB_CMD_HEARTBEAT            0x01
#define CB_CMD_ESTOP                0x02
#define CB_CMD_GET_ERROR            0x03
#define CB_CMD_RXSDO                0x04
#define CB_CMD_TXSDO                0x05
#define CB_CMD_SET_NODE_ID          0x06
#define CB_CMD_SET_AXIS_STATE       0x07
#define CB_CMD_MIT_CONTROL          0x08
#define CB_CMD_GET_ENCODER          0x09
#define CB_CMD_GET_ENCODER_COUNT    0x0A
#define CB_CMD_SET_CONTROLLER_MODE  0x0B
#define CB_CMD_SET_INPUT_POS        0x0C
#define CB_CMD_SET_INPUT_VEL        0x0D
#define CB_CMD_SET_INPUT_TORQUE     0x0E
#define CB_CMD_SET_LIMITS           0x0F
#define CB_CMD_SET_TRAJ_VEL_LIMIT   0x11
#define CB_CMD_SET_TRAJ_ACCEL       0x12
#define CB_CMD_SET_TRAJ_INERTIA     0x13
#define CB_CMD_GET_IQ               0x14
#define CB_CMD_REBOOT               0x16
#define CB_CMD_GET_BUS_VOLTAGE      0x17
#define CB_CMD_CLEAR_ERRORS         0x18
#define CB_CMD_SET_POS_GAIN         0x1A
#define CB_CMD_SET_VEL_GAINS        0x1B
#define CB_CMD_GET_TORQUES          0x1C
#define CB_CMD_SAVE_CONFIG          0x1F

/*============================================================================
 * MIT 量程参数 (可通过 SDO 运行时修改)
 *============================================================================*/
#define CB_MIT_MAX_POS          12.5f       /* rad, ±12.5 */
#define CB_MIT_MAX_VEL          65.0f       /* rad/s, ±65 */
#define CB_MIT_MAX_KP           500.0f      /* Nm/rad, 0~500 */
#define CB_MIT_MAX_KD           5.0f        /* Nm·s/rad, 0~5 */
#define CB_MIT_MAX_TORQUE       50.0f       /* Nm, ±50 */

/*============================================================================
 * Axis State 定义
 *============================================================================*/
#define CB_AXIS_STATE_IDLE              1
#define CB_AXIS_STATE_FULL_CALIB        3
#define CB_AXIS_STATE_MOTOR_CALIB       4
#define CB_AXIS_STATE_ENCODER_CALIB     7
#define CB_AXIS_STATE_CLOSED_LOOP       8
#define CB_AXIS_STATE_HOMING            11

/*============================================================================
 * Control Mode 定义 (对齐守护兽协议)
 *============================================================================*/
#define CB_CTRL_MODE_VOLTAGE    0
#define CB_CTRL_MODE_TORQUE     1
#define CB_CTRL_MODE_VELOCITY   2
#define CB_CTRL_MODE_POSITION   3

/*============================================================================
 * Input Mode 定义
 *============================================================================*/
#define CB_INPUT_MODE_INACTIVE      0
#define CB_INPUT_MODE_PASSTHROUGH   1
#define CB_INPUT_MODE_VEL_RAMP      2
#define CB_INPUT_MODE_POS_FILTER    3
#define CB_INPUT_MODE_TRAP_TRAJ     5
#define CB_INPUT_MODE_TORQUE_RAMP   6
#define CB_INPUT_MODE_MIT           9

/*============================================================================
 * 节点地址
 *============================================================================*/
#define CB_NODE_ID_MIN      1
#define CB_NODE_ID_MAX      63
#define CB_NODE_ID_DEFAULT  1

/*============================================================================
 * 超时
 *============================================================================*/
#define CB_CAN_TIMEOUT_MS   200
#define CB_MIT_TIMEOUT_MS   20
#define CB_HEARTBEAT_MS     500

/*============================================================================
 * 公共 API
 *============================================================================*/

/* 初始化: main 中 MX_FDCAN1_Init 之后调用 */
void can_cybeast_init(void);

/* 1ms tick: 超时检测 + heartbeat (放在 main loop 1ms 计时中) */
void can_cybeast_tick_1ms(void);

/* RX 分发: 从 fdcan_rx_user 调用 */
void can_cybeast_rx_dispatch(uint32_t id, const uint8_t *data, uint32_t len);

/* 主循环轮询 (预留, 当前无阻塞任务) */
void can_cybeast_poll(void);

/* 获取/设置节点地址 */
uint8_t can_cybeast_get_node_id(void);
void    can_cybeast_set_node_id(uint8_t id);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_CYBEAST_H__ */
