# 万里扬 FDCAN V1.7 协议验证流程

## 状态总览（截至 2026-05-14）

- ✅ 第 1 层 cantest1~6 全部通过
- ⏸️ 第 1 层 cantest7（超时）/ cantest8（MIT）待跑
- ⏸️ 第 2 层 USB-CAN 实测：T1 通过（含真实总线抓包），T2~T17 待做
- ⏸️ 第 3 层 与真实主站联调

## 静态分析已知疑点

| # | 文件:行 | 问题 | 影响 | 状态 |
|---|---------|------|------|------|
| 1 | `can_wly.c` `vel_int_to_rad_s` | 反馈路径误用指令路径换算，多除 GR=25 | 速度反馈缩小 25× | ✅ 已修（拆 `vel_out_to_rad_s`）|
| 2 | `can_wly.c can_wly_tick_1ms` | `CommunicateErr` 一旦置位不自愈 | 需手动 `0x701 FD` 清 | ⏸️ 调试期默认关 (`g_can_timeout_force_disable=1`) |
| 3 | `can_wly.c sdo_write_value` & `foc_api.c:514` | `temp1` 被 Rs 和 node_id 共用 | bwtest3 后 ID 丢失 | ⏸️ 待修 |
| 4 | `can_wly.c handle_mit_cmd` | MIT 的 Kp/Kd/Vel 全部丢弃 | 不符合 V1.7 完整语义 | ✅ 已修（MIT_PD_MODE 完整 PD 运算）|
| 5 | `can_wly.c pack_status_frame` | `ServoErrFlag` 32bit 截到 24bit | bit 28-30 故障丢失 | ⏸️ 待修 |

## 已完成的协议层增强

| 改动 | 文件 | 说明 |
|------|------|------|
| 状态帧 WARN 字段填充 | `can_wly.c pack_status_frame` | Bit0=MOS 过温, Bit1=电机过温 |
| 状态帧 STA Bit3=到达 | 同上 | 位置/速度/电流到达任一即置 |
| 1ms target_reach_check | `ifly_fault.c` | 按 mode 判 PositionArrived/SpeedArrived/CurrentArrived |
| CAN 超时强制关闭开关 | `can_wly.c g_can_timeout_force_disable` | 调试期默认 1, 联调改 0 |
| 使能/失能完整序列 | `handle_ctrl_frame` | 清三个指令→电流模式→`position_ref=real`→`foc_run=2`+PWM |
| 0x400 位置指令钳位 | `handle_position_cmd` | 24bit 自带量程钳位 |
| `g_can_wly_lim` ↔ Flash 软限位双向同步 | `can_wly_init` + SDO 写 0x2000/0x2001 | 协议量程改 → Flash MaxPositionLimit 跟随 |

## 协议默认量程

```c
g_can_wly_lim = {
    .pos_min = -7.0,  .pos_max = 7.0,        // rad ≈ ±401° 输出端
    .spd_min = -20.0, .spd_max = 20.0,        // rad/s ≈ ±191 rpm 输出端
    .tq_min  = -500,  .tq_max  = 500,         // N·m
    .kp_min  = 0,     .kp_max  = 500,
    .kd_min  = 0,     .kd_max  = 20,
};
```

注：`spd_min/max=±20 rad/s` 超过 `DEFAULT_MAX_SPEED=100 rpm` 硬限（10.472 rad/s），高位量程不可用。建议联调时通过 SDO 0x2002/0x2003 改为 ±2.0944 rad/s（±20 rpm）。

## cantest 命令（USART1 921600）

实现位置：`foc_bsp.c dbg_cmd_set` 的 `cantest<N>` 分支
- 入口置 `g_cantest_stub=1` + `g_can_timeout_force_disable=1` + 清 CommunicateErr
- 出口恢复

| cmd | 模拟帧 | 验证点 | 实测结果 |
|-----|--------|--------|----------|
| `cantest1` | 0x200 D=`6D 8D 01` (v_raw=36205) | velocity_ref 单位换算 | ✅ -19.39 rad/s（量程 ±20）, mode=3 |
| `cantest2` | 0x400 D=`00 00 80 00 80 01` | position_ref + Pid_PositionLimit | ✅ position_ref=0, mode=1 |
| `cantest3` | 直接调 pack_status_frame | 状态帧 12 字节解码 | ✅ POS 解码正确 |
| `cantest4` | 0x601 D=`40 00 20 00 ...` | SDO 读 0x2000 (pos_min) | ✅ 返回 -5.0f LE |
| `cantest5` | 0x601 D=`23 00 20 00 <-10.0f LE>` | SDO 写 0x2000 | ✅ pos_min 改变 + 恢复 |
| `cantest6` | 0x701 D=`FF×7 FA` | 控制帧使能 | ✅ foc_run: 0→1 |
| `cantest7` | 注帧后停 250ms | 超时保护 | ⏸️ 待跑 |
| `cantest8` | 0x500 12B MIT | MIT 解析 | ⏸️ 待跑（确认 Kp/Kd 是否丢弃）|

## 第 2 层 USB-CAN 测试矩阵（参考）

硬件：USB-CAN 适配器，1 Mbps Nominal / 5 Mbps Data，BRS 开

| 测项 | 发送帧 | 期望应答 | 状态 |
|------|--------|----------|------|
| T1 | `0x080` (任意 8B) | `0x101` 12B 状态帧 | ✅ 已通过 |
| T2 | `0x081` | `0x101` 12B | ⏸️ |
| T3 | `0x601` D=`40 00 20 00 ...` | `0x581` 含 pos_min LE | ⏸️ |
| T4 | `0x601` D=`23 00 20 00 <val>` | `0x581` ACK | ⏸️ |
| T5 | `0x601` D=`23 00 2F 00 02 00 00 00` | 改 node_id | ⏸️ |
| T6 | `0x601` D=`23 05 2F 00 02 00 00 00` | 1ms 自动上报 | ⏸️ |
| T7~T9 | 0x200/0x400/0x500 | 速度/位置/MIT 模式生效 | ⏸️ |
| T10~T13 | 0x701 FA/FB/FC/FD | 使能/失能/置零/清错 | ⏸️ |
| T14 | 停发 200ms | CommunicateErr=1（联调时开看门狗后）| ⏸️ |
| T15~T17 | 故障注入 | err1/err2/sta/warn 上报 | ⏸️ |

## 关键提醒

**联调前必做**：
1. `g_can_timeout_force_disable = 0`（恢复看门狗）
2. 通过 SDO 0x2002/0x2003 把速度量程缩到 ±2.0944 rad/s（贴近 100rpm 硬限）
3. CAN 主站心跳 ≤200ms 一次，否则触发 CommunicateErr

## 修复优先级（剩余 backlog）

| P | 问题 | 修复方向 |
|---|------|---------|
| P0 | node_id 与 Rs 共占 temp1（疑点 #3）| 移到独立 Flash 字段 |
| P1 | CommunicateErr 不自愈（疑点 #2）| 收到有效帧时同步清除 |
| P1 | ServoErrFlag 高 8 bit 截断（疑点 #5）| 协议扩展或调整故障 bit 布局 |
| P2 | MIT 阉割（疑点 #4）| 实现独立 MIT_PD_MODE，电流环前加 PD |
