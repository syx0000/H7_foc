# 万里扬 FDCAN V1.7 协议验证流程

## 状态总览（截至 2026-05-15）

- ✅ 第 1 层 cantest1~8 全部通过
- ✅ 第 2 层 USB-CAN 实测：T1~T8b + T10~T13 通过（广播/单播/SDO/速度/位置/转矩/使能/失能/置零/清错）
- ⚠️ T9 MIT 模式调试中（减速箱间隙引起极限环振荡，需要主站发当前位置或加速度斜坡）
- ⏸️ T14 看门狗、T15~T17 故障注入待做
- ⏸️ 第 3 层 与真实主站联调

## 静态分析已知疑点

| # | 文件:行 | 问题 | 影响 | 状态 |
|---|---------|------|------|------|
| 1 | `can_wly.c` `vel_int_to_rad_s` | 反馈路径误用指令路径换算，多除 GR=25 | 速度反馈缩小 25× | ✅ 已修（拆 `vel_out_to_rad_s` → 后改为 `vel_motor_to_load_rad_s`，直接用 dtheta_mech/GR）|
| 2 | `can_wly.c can_wly_tick_1ms` | `CommunicateErr` 一旦置位不自愈 | 需手动 `0x701 FD` 清 | ✅ 确认为安全设计（主站必须显式清错+重新使能）|
| 3 | `can_wly.c sdo_write_value` & `foc_api.c:514` | `temp1` 被 Rs 和 node_id 共用 | bwtest3 后 ID 丢失 | ✅ 已修（node_id 移到 temp4）|
| 4 | `can_wly.c handle_mit_cmd` | MIT 的 Kp/Kd/Vel 全部丢弃 | 不符合 V1.7 完整语义 | ✅ 已修（MIT_PD_MODE 完整 PD 运算）|
| 5 | `can_wly.c pack_status_frame` | `ServoErrFlag` 32bit 截到 24bit | bit 24-31 丢失 | ✅ 确认无需修改（丢失的均为软件诊断标志，不影响安全停机）|
| 6 | `fdcan.c` RX DLC 解析 | `(rx.DataLength >> 16)` 多 shift 一次 | 所有收帧 len=0 | ✅ 已修 |
| 7 | ISR 路径 printf 死锁 | `fdcan_rx_user` → `ClearFaults`/`Reset_objReset_Output_Encoder`/`Encoder_out_data_Reset` 含 printf | T12/T13 卡死 | ✅ 已修（删除 ISR 路径 printf）|

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
| `cantest7` | 注帧后停 250ms | 超时保护 | ✅ |
| `cantest8` | 0x500 MIT 基础 | MIT 中点值 + 中等 Kp/Kd | ✅ mode=11, Kp/Kd 生效 |
| `cantest9` | 0x500 MIT Kp 最大 | Kp=500, Kd=0 | ✅ 边界参数解析 |
| `cantest10` | 0x500 MIT Kd 最大 | Kp=0, Kd=20 | ✅ 边界参数解析 |
| `cantest11` | 0x500 MIT 正向运动 | p=+3.5rad, v=+10rad/s, Kp=50, Kd=3.8 | ✅ 正向位置/速度解析 |
| `cantest12` | 0x500 MIT 负向运动 | p=-3.5rad, v=-10rad/s, Kp=50, Kd=3.8 | ✅ 负向位置/速度解析 |
| `cantest13` | 0x500 MIT 最大正扭矩 | t=+500Nm, Kp=Kd=0 | ✅ 纯前馈正扭矩 |
| `cantest14` | 0x500 MIT 最大负扭矩 | t=-500Nm, Kp=Kd=0 | ✅ 纯前馈负扭矩 |
| `cantest15` | 0x500 MIT 位置上限 | p=+7rad (pos_max) | ✅ 边界位置解析 |
| `cantest16` | 0x500 MIT 位置下限 | p=-7rad (pos_min) | ✅ 边界位置解析 |
| `cantest17` | 0x500 MIT 速度上限 | v=+20rad/s (spd_max) | ✅ 边界速度解析 |
| `cantest18` | 0x500 MIT 速度下限 | v=-20rad/s (spd_min) | ✅ 边界速度解析 |
| `cantest19` | 0x500 MIT 典型阻抗 | Kp=100, Kd=5, t=+10Nm | ✅ 典型阻抗控制参数 |

## 第 2 层 USB-CAN 测试矩阵（参考）

硬件：USB-CAN 适配器，1 Mbps Nominal / 5 Mbps Data，BRS 开

| 测项 | 发送帧 | 期望应答 | 状态 |
|------|--------|----------|------|
| T1 | `0x080` (任意 8B) | `0x101` 12B 状态帧 | ✅ |
| T2 | `0x081` | `0x101` 12B | ✅ |
| T3 | `0x601` D=`40 00 20 00 00 00 00 00` | `0x581` 含 pos_min LE | ✅ |
| T4 | `0x601` D=`23 00 20 00 00 00 20 C1` | `0x581` ACK (写 pos_min=-10.0f) | ✅ |
| T5 | `0x601` D=`23 00 2F 00 02 00 00 00` | 改 node_id→2 | ✅ |
| T6 | `0x601` D=`23 05 2F 00 02 00 00 00` | 1ms 自动上报 | ✅ |
| T7 | `0x200` D=`FF 9F 01` | 速度 +5 rad/s | ✅ |
| T8 | `0x400` D=`00 00 80 FF 9F 01` | 位置 0 rad, 限速 5 rad/s | ✅ |
| T8b | `0x300` D=`41 80 01` | 转矩 +1 N·m | ✅ |
| T9 | `0x500` D=`00 00 80 00 80 00 80 47 05 99 19 01` | MIT Kp=10 Kd=2 目标 0 rad (FD帧) | ⚠️ |
| T10 | `0x701` D=`FF FF FF FF FF FF FF FA` | 使能 | ✅ |
| T11 | `0x701` D=`FF FF FF FF FF FF FF FB` | 失能 | ✅ |
| T12 | `0x701` D=`FF FF FF FF FF FF FF FC` | 置零 | ✅ |
| T13 | `0x701` D=`FF FF FF FF FF FF FF FD` | 清错 | ✅ |
| T14 | 停发 200ms | CommunicateErr=1（联调时开看门狗后）| ⏸️ |
| T15~T17 | 故障注入 | err1/err2/sta/warn 上报 | ⏸️ |

### 报文详解

#### T3 SDO 读 pos_min
```
TX: ID=0x601  DLC=8  Data: 40 00 20 00 00 00 00 00
RX: ID=0x581  DLC=8  Data: 60 00 20 00 <pos_min float32 LE>
```
- D[0]=0x40 读请求, D[1..2]=0x2000 索引, D[3]=0x00 子索引
- 响应 D[4..7] = float LE, 如 -7.0f → `00 00 E0 C0`

#### T4 SDO 写 pos_min = -10.0f
```
TX: ID=0x601  DLC=8  Data: 23 00 20 00 00 00 20 C1
RX: ID=0x581  DLC=8  Data: 60 00 20 00 00 00 20 C1
```
- D[0]=0x23 写请求, D[4..7] = -10.0f LE = `00 00 20 C1`
- 恢复默认: `23 00 20 00 00 00 E0 C0` (-7.0f)

#### T7 速度指令 (0x200, 3B/槽)
```
格式: V_des[7:0] V_des[15:8] CANID
量程: spd_min=-20, spd_max=+20 rad/s, 中点 0x7FFF=0
公式: v_raw = (target_rad_s + 20) / 40 × 65535
```
| 目标 | v_raw | Data |
|------|-------|------|
| 0 rad/s (停) | 0x7FFF | `FF 7F 01` |
| +5 rad/s (47.7rpm) | 0x9FFF | `FF 9F 01` |
| +10 rad/s | 0xBFFF | `FF BF 01` |
| -5 rad/s | 0x5FFF | `FF 5F 01` |
| +20 rad/s (满) | 0xFFFF | `FF FF 01` |

#### T8 位置指令 (0x400, 6B/槽)
```
格式: POS[7:0] POS[15:8] POS[23:16] V[7:0] V[15:8] CANID
量程: pos_min=-7, pos_max=+7 rad, 中点 0x800000=0
公式: p_raw = (target_rad + 7) / 14 × 16777215
速度限制: 实际 = min(CAN_v_raw, MaxSpeed=100rpm)
```
| 目标位置 | p_raw | Data (v=5rad/s) |
|---------|-------|-----------------|
| 0 rad (中心) | 0x800000 | `00 00 80 FF 9F 01` |
| +1 rad (57.3°) | 0x924924 | `24 49 92 FF 9F 01` |
| -1 rad | 0x6DB6DB | `DB B6 6D FF 9F 01` |
| +0.5 rad | 0x892492 | `92 24 89 FF 9F 01` |
| +3.5 rad (半量程) | 0xC00000 | `00 00 C0 FF 9F 01` |

#### T8b 转矩指令 (0x300, 3B/槽)
```
格式: T_des[7:0] T_des[15:8] CANID
量程: tq_min=-500, tq_max=+500 N·m, 中点 0x7FFF=0
公式: t_raw = (target_nm + 500) / 1000 × 65535
注意: g_can_wly_kt_out=1.0 时 1 N·m = 1A = I_q_ref=1024(Q10)
```
| 目标 | t_raw | Data |
|------|-------|------|
| 0 N·m | 0x7FFF | `FF 7F 01` |
| +0.1 N·m | 0x8006 | `06 80 01` |
| -0.1 N·m | 0x7FF9 | `F9 7F 01` |
| +1.0 N·m | 0x8041 | `41 80 01` |
| -1.0 N·m | 0x7FBE | `BE 7F 01` |
| +5.0 N·m | 0x8147 | `47 81 01` |

⚠️ 转矩模式无速度环保护，空载会持续加速。测完立即发 `FF 7F 01` 或 T11 失能。

#### T9 MIT 指令 (0x500, 12B/槽, 需 CAN-FD)
```
格式: POS[23:0] VEL[15:0] T[15:0] Kp[15:0] Kd[15:0] CANID
量程: pos ±7rad, vel ±20rad/s, tq ±500N·m, Kp 0~500, Kd 0~20
控制律: Iq = Kp×(p_des-pos) + Kd×(v_des-vel) + t_ff
```
| Kp | raw | LE | Kd | raw | LE |
|----|-----|----|----|-----|-----|
| 5 | 0x0666 | `66 06` | 1 | 0x0CCC | `CC 0C` |
| 10 | 0x0CCC | `CC 0C` | 2 | 0x1999 | `99 19` |
| 50 | 0x1999 | `99 19` | 3 | 0x2666 | `66 26` |
| 100 | 0x3333 | `33 33` | 5 | 0x3FFF | `FF 3F` |
| 200 | 0x6666 | `66 66` | 10 | 0x7FFF | `FF 7F` |

示例 (Kp=10, Kd=2, 目标 0 rad, 无前馈):
```
ID=0x500  DLC=12 (CAN-FD)
Data: 00 00 80 00 80 00 80 CC 0C 99 19 01
```

⚠️ MIT 目标位置应设为当前位置附近，否则阶跃冲击。先发 T1 读状态帧 D[0..2] 获取当前 p_raw。

#### T10~T13 控制帧 (0x701, 8B)
```
格式: D[0..6]=0xFF, D[7]=命令码
```
| 命令 | D[7] | Data | 效果 |
|------|------|------|------|
| 使能 | 0xFA | `FF FF FF FF FF FF FF FA` | foc_run=2, PWM 开 |
| 失能 | 0xFB | `FF FF FF FF FF FF FF FB` | foc_run=0, PWM 关, I_q/vel=0 |
| 置零 | 0xFC | `FF FF FF FF FF FF FF FC` | 当前位置设为零点, 写 Flash |
| 清错 | 0xFD | `FF FF FF FF FF FF FF FD` | ServoErrFlag=0, 恢复 MOE |

#### SDO 常用对象字典
| 索引 | 名称 | 类型 | 默认值 |
|------|------|------|--------|
| 0x2000 | pos_min | float | -7.0 rad |
| 0x2001 | pos_max | float | 7.0 rad |
| 0x2002 | spd_min | float | -20.0 rad/s |
| 0x2003 | spd_max | float | 20.0 rad/s |
| 0x2004 | tq_min | float | -500.0 N·m |
| 0x2005 | tq_max | float | 500.0 N·m |
| 0x2F00 | node_id | uint8 | 1 |
| 0x2F05 | auto_report | uint8 | 0 (2=开启) |

#### float LE 速查
| 值 | LE 字节 |
|----|--------|
| -10.0f | `00 00 20 C1` |
| -7.0f | `00 00 E0 C0` |
| -5.0f | `00 00 A0 C0` |
| -2.0944f | `DB 0F 06 C0` |
| 0.0f | `00 00 00 00` |
| 2.0944f | `DB 0F 06 40` |
| 5.0f | `00 00 A0 40` |
| 7.0f | `00 00 E0 40` |
| 10.0f | `00 00 20 41` |
| 20.0f | `00 00 A0 41` |

## MIT 实测验证序列

主站周期 ≤10ms 重发 0x500（`MIT_TIMEOUT_MS=20` ([can_wly.c:47](Core/Src/can_wly.c#L47)) 超时退回 IDLE_MODE）。串口 `logid 100/50/40` 观察位置/速度/电流。所有 raw 由 `float_to_uint(x, x_min, x_max, bits)` ([can_wly.c:62-68](Core/Src/can_wly.c#L62-L68)) 默认量程实算。

### Step 0 安全态准备（0x701 / 8B 经典 CAN）

| 顺序 | 用途 | Data |
|---|---|---|
| 0.1 | 失能 | `FF FF FF FF FF FF FF FB` |
| 0.2 | 清错 | `FF FF FF FF FF FF FF FD` |
| 0.3 | 置零 | `FF FF FF FF FF FF FF FC` |
| 0.4 | 使能 | `FF FF FF FF FF FF FF FA` |

### Step 1~7 MIT 帧（ID=0x500，DLC=12，CAN-FD）

| Step | 用途 | p_des | v_des | t_ff | Kp | Kd | Data |
|------|------|-------|-------|------|----|----|------|
| 1 | 切模式不动作 | 0 | 0 | 0 | 0 | 0 | `FF FF 7F  FF 7F  FF 7F  00 00  00 00  01` |
| 2 | 位置保持 | 0 | 0 | 0 | 10 | 1 | `FF FF 7F  FF 7F  FF 7F  1E 05  CC 0C  01` |
| 3 | 阶跃 +5° | +0.0873 | 0 | 0 | 10 | 1 | `B2 9A 81  FF 7F  FF 7F  1E 05  CC 0C  01` |
| 4 | 阶跃 −5° | −0.0873 | 0 | 0 | 10 | 1 | `4C 65 7E  FF 7F  FF 7F  1E 05  CC 0C  01` |
| 5 | 提刚度 +5° | +0.0873 | 0 | 0 | 50 | 2 | `B2 9A 81  FF 7F  FF 7F  99 19  99 19  01` |
| 6 | 速度跟随 | 0 | +0.5 rad/s | 0 | 0 | 2 | `FF FF 7F  32 83  FF 7F  00 00  99 19  01` |
| 7 | 转矩前馈 | 0 | 0 | +0.2 N·m | 0 | 0 | `FF FF 7F  FF 7F  19 80  00 00  00 00  01` |
| 收尾 | 失能 | — | — | — | — | — | 0x701 `FF FF FF FF FF FF FF FB` |

### 验收点

- **Step 1**: `controller_mode==4` (MIT_PD_MODE)，I_q ≈ 0
- **Step 2**: 手扭输出端有回弹，松手回 0 位 ±0.5°（减速箱间隙限制）
- **Step 3/4**: 输出端转 ±5°，超调 ≤20%，稳态误差 ≤0.5°
- **Step 5**: 响应更快、刚度更高；如 ±0.1° 持续抖动 → 减速箱间隙激发极限环，降 Kp 或加 Kd
- **Step 6**: 输出端 ≈4.77 rpm 转动，v_des 改回 0 即停
- **Step 7**: ⚠️ 与 0x300 转矩模式同样无速度环保护，空载持续加速。看到电流稳定立即回 Step 1 或失能

## 关键提醒

**联调前必做**：
1. `g_can_timeout_force_disable = 0`（恢复看门狗）
2. 通过 SDO 0x2002/0x2003 把速度量程缩到 ±2.0944 rad/s（贴近 100rpm 硬限）
3. CAN 主站心跳 ≤200ms 一次，否则触发 CommunicateErr

## 修复优先级（剩余 backlog）

所有原始疑点已关闭：

| P | 问题 | 状态 |
|---|------|------|
| ~~P0~~ | ~~node_id 与 Rs 共占 temp1（疑点 #3）~~ | ✅ node_id 移到 temp4 |
| ~~P1~~ | ~~CommunicateErr 不自愈（疑点 #2）~~ | ✅ 确认为安全设计，不改 |
| ~~P1~~ | ~~ServoErrFlag 高 8 bit 截断（疑点 #5）~~ | ✅ 丢失 bit 均为软件诊断，不影响安全 |
| ~~P2~~ | ~~MIT 阉割（疑点 #4）~~ | ✅ MIT_PD_MODE 完整实现 |

### 待优化项（非阻塞）

| P | 问题 | 方向 |
|---|------|------|
| P2 | MIT 减速箱间隙振荡 | 主站侧发当前位置 / 固件加输出斜坡限制 |
| P2 | 转矩模式无超速保护 | 加速度超 MaxSpeed 时自动减 I_q_ref |

### 待实现 SDO 条目（参考"小狗自定义参数命令202603292034-1.xls"）

| 索引 | 名称 | 类型 | R/W | 说明 | 优先级 |
|------|------|------|-----|------|--------|
| 0x2F01 | 电机极对数 | Uint8 | RW | 写入后更新 NPP（4=12极, 5=17.131极）| P1 |
| 0x2F02 | 编码器标定 | Uint8 | W | 0=无操作, 1=触发电角度标定（等效串口 `Cali`）| P1 |
| 0x2F03 | 位置环运行模式 | Uint8 | RW | 0=CSP 模式, 1=PP 测试模式（用于位置精度测试）| P2 |
| 0x2F04 | 软件版本号 | Uint16 | RO | 返回 SOFT_VERSION 编码 | P2 |
| 0x2F06 | 电流带宽测试频率 | Uint32 | RW | ✅ 已实现 注入频率 Hz, 钳 [1, 5000] | P2 |
| 0x2F07 | 电流带宽测试幅值 | Uint32 | RW | ✅ 已实现 注入幅值 Q10 A, 钳 [0, 30A] | P2 |

### 帧 ID 实现状态

| 帧 ID | 方向 | DLC | 说明 | 状态 |
|--------|------|-----|------|------|
| 0x7FD | 从站→主站 | 8 | ✅ **已实现** 逐拍数据 (byte0-3=Iq_ref_filterd_A×1000+50000, byte4-7=I_q_A×1000+50000)，实际 100µs/帧 (FOC=10kHz) | P2 |
| 0x7FE | 从站→主站 | 16 | ✅ 已实现 扩展状态帧（1ms 周期） | P1 |

### 0x2F05 测试命令实现 (2026-05)

| cmd | 协议语义 | 实现 |
|-----|---------|------|
| 0 | 无操作 / 停止 | ✅ 停止 0x7FD 流 |
| 1 | 启动电流带宽测试 | ✅ 按 0x2F06 频率 + 0x2F07 幅值在 ISR 注入单频正弦, 同时 100µs/帧发 0x7FD |
| 2 | 反馈测试状态上传 | ⚠ 私有扩展: 开启 1ms 周期 0x7FE 主动上报 (与协议字面"上报测试结果"语义不同, 改协议或加新 OD 待定) |

### Kt LUT (2026-05)

`g_can_wly_kt_out` 从单一缩放因子升级为查表+插值:
- `can_wly_iA_to_Nm(i_A)` / `can_wly_Nm_to_iA(t_Nm)` 双向接口
- 10 点 LUT (占位线性, 标定后替换)
- `g_can_wly_kt_out` 保留作整体缩放
- 串口扭矩指令 (`Runcmd...tar...`) 也走 LUT, tar 字段语义改 N·m
