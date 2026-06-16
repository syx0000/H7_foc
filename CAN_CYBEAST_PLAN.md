# 守护兽 CAN Simple 协议适配 - 实现与验证计划

> 状态: **Phase 1~3 全部实现完成**, 编译通过 (0 Error 0 Warning), 待实机验证

---

## 编译切换方案

```c
// Core/Inc/can_protocol_sel.h
#define CAN_PROTO_WLY       0   // 万里扬 V1.7 (CAN-FD + BRS, 1M+5M)
#define CAN_PROTO_CYBEAST   1   // 守护兽 CAN Simple (Classic CAN, 1M)

#define CAN_PROTOCOL_SEL    CAN_PROTO_CYBEAST   // ← 切换这里
```

全工程通过 `#if (CAN_PROTOCOL_SEL == CAN_PROTO_CYBEAST)` 条件编译。两套协议互斥，零运行时开销。

---

## 文件清单

| 文件 | 类型 | 说明 |
|------|------|------|
| `Core/Inc/can_protocol_sel.h` | 新建 | 协议选择宏 |
| `Core/Src/can_cybeast.c` | 新建 | 守护兽协议完整实现 (~820行) |
| `Core/Inc/can_cybeast.h` | 新建 | CMD ID / 量程 / 状态 / API |
| `Core/Src/fdcan.c` | 改动 | fdcan_send() Classic CAN + Init FrameFormat |
| `Core/Src/can_wly.c` | 改动 | fdcan_rx_user() dispatch 分支 |
| `Core/Src/main.c` | 改动 | init/tick/poll 切换 |
| `Core/Src/can_debug.c` | 改动 | CAN log 帧 8B 限制 |

**不改的文件:** FOC 内核全部、encoder.c、adc.c、tim.c、flash_port.c、usart.c

---

## 协议实现覆盖率: 31/31

| CMD | 名称 | 方向 | 状态 | 说明 |
|-----|------|------|------|------|
| 0x01 | Heartbeat | TX 500ms | ✅ | Axis_Error + State + Flags + Temp + Life |
| 0x02 | Estop | RX | ✅ | 紧急停机 + 置 CommunicateErr |
| 0x03 | Get_Error | RX→TX | ✅ | 分类错误码 (motor/encoder/ctrl/sys) |
| 0x04 | RxSdo | RX | ✅ | 通用 endpoint 读写 (10 个端点) |
| 0x05 | TxSdo | TX | ✅ | RxSdo 响应帧 |
| 0x06 | Set_Axis_Node_ID | RX | ✅ | 运行时改 ID + Flash 持久化 |
| 0x07 | Set_Axis_State | RX | ✅ | IDLE/CLOSED/FULL_CALIB/MOTOR_CALIB/ENCODER |
| 0x08 | MIT_Control | RX/TX | ✅ | 8B 紧凑编码 + 6B 实际反馈 |
| 0x09 | Get_Encoder_Estimates | RX→TX | ✅ | 转子侧 float rev + float rev/s |
| 0x0A | Get_Encoder_Count | RX→TX | ✅ | shadow_count + count_in_cpr |
| 0x0B | Set_Controller_Mode | RX | ✅ | 力矩/速度/位置 + input_mode |
| 0x0C | Set_Input_Pos | RX | ✅ | float rev + int16 vel_ff + int16 torque_ff |
| 0x0D | Set_Input_Vel | RX | ✅ | float rev/s + float torque_ff |
| 0x0E | Set_Input_Torque | RX | ✅ | float Nm → Kt LUT → I_q_ref |
| 0x0F | Set_Limits | RX | ✅ | vel_limit → DEFAULT_MAX_SPEED, current_limit → PID OutputMax |
| 0x10 | Start_Anticogging | - | N/A | 无齿槽补偿硬件 |
| 0x11 | Set_Traj_Vel_Limit | RX | ✅ | → SmoothPosRef.v_max |
| 0x12 | Set_Traj_Accel_Limits | RX | ✅ | → SmoothPosRef.a_max |
| 0x13 | Set_Traj_Inertia | RX | ✅ | 存储预留 |
| 0x14 | Get_Iq | RX→TX | ✅ | float Iq_set + float Iq_meas |
| 0x15 | Get_Temperature | RX→TX | ✅ | float motor_temp + float fet_temp |
| 0x16 | Reboot | RX | ✅ | NVIC_SystemReset |
| 0x17 | Get_Bus_Voltage_Current | RX→TX | ✅ | float Vbus + float Ibus |
| 0x18 | Clear_Errors | RX | ✅ | ServoErrFlag = 0 |
| 0x19 | Set_Move_Incremental | RX | ✅ | 转子侧 rev 增量 → position_ref 累加 |
| 0x1A | Set_Pos_Gain | RX | ✅ | float → IncPID_Position.Kp |
| 0x1B | Set_Vel_Gains | RX | ✅ | float Kp + float Ki → IncPID_Speed |
| 0x1C | Get_Torques | RX→TX | ✅ | float Torque_Setpoint + float Torque |
| 0x1D | Get_Powers | RX→TX | ✅ | float Electrical + float Mechanical |
| 0x1E | Disable_Can | RX | ✅ | HAL_FDCAN_Stop + Reset |
| 0x1F | Save_Configuration | RX | ✅ | Flash Erase+Write + Reset |

---

## 三环控制使用方法

### 力矩控制
```
1. Set_Axis_State(8)                    → 闭环使能
2. Set_Controller_Mode(1, 1)            → 力矩模式 + 直接输入
3. Set_Input_Torque(float Nm)           → 目标力矩
```
链路: `Nm → Kt LUT → I_q_ref → 电流环PI → V_q → SVPWM`

### 速度控制
```
1. Set_Axis_State(8)                    → 闭环使能
2. Set_Controller_Mode(2, 2)            → 速度模式 + 速度斜坡
3. Set_Input_Vel(float rev/s, float Nm) → 目标速度 + 力矩前馈
```
链路: `rev/s → velocity_ref → 速度环(5kHz) → I_q_ref → 电流环 → SVPWM`

### 位置控制
```
1. Set_Axis_State(8)                    → 闭环使能
2. Set_Controller_Mode(3, 5)            → 位置模式 + 梯形曲线
3. Set_Traj_Vel_Limit(float rev/s)      → 梯形规划速度限制 (可选)
4. Set_Traj_Accel_Limits(float rev/s²)  → 梯形规划加速度 (可选)
5. Set_Input_Pos(float rev, vel_ff, tq_ff) → 目标位置
```
链路: `rev → 梯形规划 → 位置环(2.5kHz) → 速度环 → 电流环 → SVPWM`

### MIT 阻抗控制 (无需 Set_Controller_Mode)
```
1. Set_Axis_State(8)                    → 闭环使能
2. MIT_Control(pos, vel, kp, kd, torque) → 自动切 MIT_PD_MODE
```
链路: `I_q = Kp×(p_des-p_fb) + Kd×(v_des-v_fb) + T_ff → 电流环 → SVPWM`

---

## CAN ID 编码

```
CAN_ID = (node_id << 5) | cmd_id
  node_id: bits[10:5], 范围 1~63
  cmd_id:  bits[4:0],  范围 0x00~0x1F
```

例: node_id=5, MIT_Control → CAN_ID = (5<<5)|0x08 = 0xA8

---

## MIT 帧格式

### 输入 (主机→电机, 8B, Big-Endian bit-packed)
```
BYTE0-1: position  16bit → [-12.5, +12.5] rad (输出轴侧)
BYTE2高8+BYTE3高4: velocity  12bit → [-65, +65] rad/s
BYTE3低4+BYTE4:    Kp        12bit → [0, 500] Nm/rad
BYTE5高8+BYTE6高4: Kd        12bit → [0, 5] Nm·s/rad
BYTE6低4+BYTE7:    torque    12bit → [-50, +50] Nm
```

### 输出 (电机→主机, 8B, 6B有效)
```
BYTE0:             node_id   8bit
BYTE1-2:           position  16bit (实际编码器位置)
BYTE3高8+BYTE4高4: velocity  12bit (实际速度)
BYTE4低4+BYTE5:    torque    12bit (实际力矩)
BYTE6-7:           0x00 (padding)
```

### 量程参数 (可通过 RxSdo 0x0100~0x0104 运行时修改)
```c
mit_max_pos    = 12.5f    // rad
mit_max_vel    = 65.0f    // rad/s
mit_max_kp     = 500.0f   // Nm/rad
mit_max_kd     = 5.0f     // Nm·s/rad
mit_max_torque = 50.0f    // Nm
```

---

## RxSdo Endpoint 表

| EP_ID | 参数 | 类型 | R/W |
|-------|------|------|-----|
| 0x0001 | vbus_voltage | float | RO |
| 0x0002 | gear_ratio | float | RW |
| 0x0010 | pole_pairs | uint32 | RO |
| 0x0020 | torque_constant (Kt) | float | RO |
| 0x0100 | mit_max_pos | float | RW |
| 0x0101 | mit_max_vel | float | RW |
| 0x0102 | mit_max_kp | float | RW |
| 0x0103 | mit_max_kd | float | RW |
| 0x0104 | mit_max_torque | float | RW |
| 0x0200 | node_id | uint8 | RW |

---

## 单位映射

| 协议单位 | 内部单位 | 换算 |
|---------|---------|------|
| rev (输出端) | 1°/1024 LSB | `internal = rev × 360 × 1024` |
| rev/s (输出端) | rpm×1024×25 | `internal = revs × 60 × 1024 × 25` |
| rad (输出端) | 1°/1024 LSB | `internal = rad × (180/π) × 1024` |
| rad/s (输出端) | rpm×1024×25 | `internal = rads × (30/π) × 1024 × 25` |
| Nm (输出端) | Q10 A (电机端) | `I_q_ref = Kt_LUT(Nm)` |
| A (float) | Q10 | `q10 = A × 1024` |
| rev (转子侧, Get_Encoder) | 1°/1024 | `rev_rotor = pos_out × GR / (360×1024)` |
| rev/s (转子侧, Get_Encoder) | rpm×1024 | `revs_rotor = dtheta_mech / (1024×60)` |

---

## 状态机映射

| 守护兽 State | 值 | MCU 动作 |
|---|---|---|
| IDLE | 1 | foc_run=0, PWM off |
| FULL_CALIBRATION | 3 | Rs/Ld/Lq + 电角度辨识 |
| MOTOR_CALIB | 4 | Rs/Ld/Lq 辨识 |
| ENCODER_CALIB | 7 | 电角度辨识 |
| CLOSED_LOOP | 8 | foc_run=2, PWM on |

---

## 超时保护

| 机制 | 超时 | 动作 |
|------|------|------|
| CAN 通信超时 | 200ms 无任何帧 | 置 CommunicateErr 故障 |
| MIT 超时 | 20ms 无 MIT 帧 | 退出 MIT → 零电流 + 故障 |

---

## CAN 总线配置

| 参数 | 守护兽模式 | WLY 模式 |
|------|-----------|---------|
| 帧格式 | Classic CAN | CAN-FD + BRS |
| 仲裁段波特率 | 1 Mbps | 1 Mbps |
| 数据段波特率 | N/A | 5 Mbps |
| 最大 DLC | 8 字节 | 64 字节 |
| FrameFormat | FDCAN_FRAME_CLASSIC | FDCAN_FRAME_FD_BRS |

---

## 与万里扬协议的兼容性

- 编译期互斥 (`#if` 宏切换), 不可同时运行
- Flash 存储共享 (node_id 用 temp4 低字节, 两种协议读同一位置)
- Kt LUT 共享 (`can_wly_Nm_to_iA` / `can_wly_iA_to_Nm`)
- FOC 内核完全共享 (三环PID / SVPWM / 编码器 / ADC 零改动)
- CAN 调试通道 (0x7E0-0x7EF) 两种模式都保留 (命令/响应≤8B可用, CAN log 帧禁止)
- 串口调试通道完全不受影响

---

## 验证计划

### V1: 编译验证 ✅ 已通过
- [x] `CAN_PROTO_WLY` 编译 0 Error 0 Warning
- [x] `CAN_PROTO_CYBEAST` 编译 0 Error 0 Warning

### V2: 串口冒烟 (无需 CAN 适配器)
- [ ] 切 CYBEAST 编译 → 烧录 → 串口开机 log 正常
- [ ] 打印 "CyberBeast CAN Simple init, node_id=1"
- [ ] 串口命令 `logid50` / `logfreq100` 正常工作
- [ ] 电机闭环运行正常

### V3: CAN 心跳验证
- [ ] USB-CAN 适配器 (Classic CAN 1Mbps) 连接
- [ ] 每 500ms 收到 ID = `(1<<5)|0x01` = 0x21 的 8B 帧
- [ ] byte4=8(闭环), byte7 递增(life)

### V4: MIT 控制验证
- [ ] 发 MIT 命令帧 (ID=0x28, 8B) → 收到反馈帧 (ID=0x28, 8B)
- [ ] 全零帧 → 零力矩保持
- [ ] pos=0, kp=小值, kd=小值 → 弹簧效应
- [ ] 停发 → 20ms 超时退出

### V5: 三环控制验证
- [ ] 力矩: Set_Controller_Mode(1,1) + Set_Input_Torque(1.0) → 恒力矩
- [ ] 速度: Set_Controller_Mode(2,2) + Set_Input_Vel(1.0) → 匀速
- [ ] 位置: Set_Controller_Mode(3,5) + Set_Input_Pos(1.0) → 定位

### V6: 查询帧验证
- [ ] Get_Encoder → float pos + float vel 与串口一致
- [ ] Get_Iq → float Iq_set + float Iq_meas
- [ ] Get_Bus_Voltage → float Vbus

### V7: 参数配置验证
- [ ] Set_Vel_Gains → 串口确认生效
- [ ] Set_Limits → 限幅生效
- [ ] Save_Configuration → 重启后参数保持

### V8: 边界与鲁棒性
- [ ] 错误 node_id → 不响应
- [ ] DLC < 8 → 不崩溃
- [ ] 1ms MIT 连续 → 无丢帧
- [ ] 拔线 → 200ms 超时报错
- [ ] Clear_Errors → 恢复

---

## Python 上位机示例

```python
import can, struct, time

bus = can.Bus(channel='com3', interface='slcan', bitrate=1000000)
NODE_ID = 1

def send(cmd, data=b'\x00'*8):
    msg = can.Message(arbitration_id=(NODE_ID<<5)|cmd, data=data, is_extended_id=False)
    bus.send(msg)

def recv(timeout=0.5):
    return bus.recv(timeout)

# 使能
send(0x07, struct.pack('<I', 8) + b'\x00'*4)
time.sleep(0.1)

# 速度模式: 1 rev/s
send(0x0B, struct.pack('<II', 2, 2))       # velocity + vel_ramp
send(0x0D, struct.pack('<ff', 1.0, 0.0))   # 1.0 rev/s

# 读编码器
send(0x09, b'\x00'*8)
msg = recv()
if msg:
    pos, vel = struct.unpack('<ff', bytes(msg.data))
    print(f"pos={pos:.3f} rev, vel={vel:.3f} rev/s")

# 停机
send(0x07, struct.pack('<I', 1) + b'\x00'*4)
```
