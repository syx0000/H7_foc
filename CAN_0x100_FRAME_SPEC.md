# 0x100+节点号 状态帧格式说明

## 概述

0x100+节点号 (0x101~0x17F) 是万里扬 CAN-FD 协议中从站的主动/回复状态帧，包含完整的位置、速度、转矩反馈和指令通道。

**帧长**: 33 字节 (FDCAN 自动 padding 到 48B DLC)  
**更新周期**: 
- 主动上报模式 (0x2F05=2): 1ms 周期
- 被动模式: 收到 0x080 广播查询或指令帧时立即回复

## 字节映射表

| 字节偏移 | 字段名 | 类型 | 单位/量程 | 说明 |
|---------|--------|------|----------|------|
| D[0..2] | Pact | uint24 LE | 标幺化 (0~2²³-1) | 位置反馈，映射到 [PosMin, PosMax] rad |
| D[3..4] | Vact | uint16 LE | 标幺化 (0~2¹⁶-1) | 速度反馈，映射到 [SpdMin, SpdMax] rad/s |
| D[5..6] | Tact | uint16 LE | 标幺化 (0~2¹⁶-1) | 转矩反馈，映射到 [TqMin, TqMax] N·m |
| D[7..8] | Err1 | uint16 LE | 位掩码 | 故障字低 16 位 (ServoErrFlag.All_Flag) |
| D[9..10] | Err2 | uint16 LE | 位掩码 | 故障字高 16 位 (保留) |
| D[11] | warn | uint8 | 位掩码 | 警告标志: Bit0=MOS过温, Bit1=电机过温 |
| D[12] | STA | uint8 | 位掩码 | 状态字: Bit0=使能, Bit1=故障, Bit2=警告, Bit3=到达 |
| D[13..15] | Pcmd | uint24 LE | 标幺化 (0~2²³-1) | 位置指令，映射到 [PosMin, PosMax] rad |
| D[16..17] | Vcmd | uint16 LE | 标幺化 (0~2¹⁶-1) | 速度指令，映射到 [SpdMin, SpdMax] rad/s |
| D[18..19] | Tcmd | uint16 LE | 标幺化 (0~2¹⁶-1) | 转矩指令 (最近一次 0x300/0x500 下发值) |
| D[20..21] | iqref | uint16 LE | 0.01A + 偏置 10000 | Iq 指令，量程 -100A ~ +555A |
| D[22..23] | iqfdb | uint16 LE | 0.01A + 偏置 10000 | Iq 反馈 (滤波后)，量程 -100A ~ +555A |
| D[24..25] | Irms | uint16 LE | 0.01A + 偏置 10000 | 电流有效值 √((Id² + Iq²)/2) 真实 RMS |
| D[26..27] | MIT_T | uint16 LE | 标幺化 (0~2¹⁶-1) | 实际输出扭矩 (I_q_ref → N·m via LUT, 含 MIT 解算 / PI 输出) |
| D[28] | Vdc | uint8 | V | 母线电压 (直接值，量程 0~255V) |
| D[29..30] | Temp_D | int16 LE | 0.1°C | 驱动板温度 (有符号) |
| D[31..32] | Temp_M | int16 LE | 0.1°C | 电机温度 (有符号) |

## 标幺化映射公式

### 位置变量 (Pact / Pcmd)
```
编码: raw = (val - PosMin) / (PosMax - PosMin) * (2^23 - 1)
解码: val = PosMin + raw * (PosMax - PosMin) / (2^23 - 1)
```
- 量程: [PosMin, PosMax] rad (默认 -7 ~ +7 rad)
- 精度: 23 bit → ~0.83 μrad/LSB (约 0.00005°)

### 速度变量 (Vact / Vcmd)
```
编码: raw = (val - SpdMin) / (SpdMax - SpdMin) * (2^16 - 1)
解码: val = SpdMin + raw * (SpdMax - SpdMin) / (2^16 - 1)
```
- 量程: [SpdMin, SpdMax] rad/s (默认 -20 ~ +20 rad/s)
- 精度: 16 bit → ~0.00061 rad/s/LSB (约 0.0058 rpm)

### 转矩变量 (Tact / Tcmd / MIT_T)
```
编码: raw = (val - TqMin) / (TqMax - TqMin) * (2^16 - 1)
解码: val = TqMin + raw * (TqMax - TqMin) / (2^16 - 1)
```
- 量程: [TqMin, TqMax] N·m (默认 -500 ~ +500 N·m)
- 精度: 16 bit → ~0.015 N·m/LSB

## 电流字段 (iqref / iqfdb / Irms)

### 映射公式
```
编码: raw = I_A * 100 + 10000
解码: I_A = (raw - 10000) / 100
```
- 量程: -100A ~ +555A (uint16 = 0 ~ 65535)
- 精度: 0.01A/LSB (10mA)
- **用途**: 
  - iqref: 指令侧 Iq (来自位置环/速度环/MIT 输出)
  - iqfdb: 反馈侧 Iq (一阶 LPF 滤波，仅供上报，控制环不用)
  - Irms: 电流有效值，采用 √((Id² + Iq²)/2) 计算真实 RMS

### 注意事项
1. **偏置 10000**: 支持负电流 (制动/发电工况)，0A 对应 raw=10000
2. **与旧版差异**: 旧版用 int16 0.01A (量程 ±327A)，新版用 uint16 + 偏置扩大量程
3. **与 Kt LUT 独立**: iqref/iqfdb 是电流值，上位机要扭矩需自行乘 Kt。Tact/Tcmd/MIT_T 已经是扭矩值 (经 Kt LUT 映射)

## 母线电压 (Vdc)

### 采集链路
```
ADC → 平均 (2 次采样) → 分压比还原 → uint8
```

### 公式
```c
V_bus = g_vdc_raw * 33 * 21 / 65535 / 10;  // 分压比 21:1, ADC 3.3V 满量程
raw_u8 = (V_bus > 255) ? 255 : V_bus;     // 限幅到 uint8
```

- 量程: 0 ~ 255V (直接值)
- 精度: 1V/LSB
- **不适用于高压母线**: 若母线 >255V (如工业 400V DC)，需改用 uint16 或缩放因子

## 温度字段 (Temp_D / Temp_M)

### 映射公式
```
编码: raw = T_degC * 10
解码: T_degC = raw / 10
```
- 类型: int16 (有符号)，支持负温度
- 量程: -3276.8°C ~ +3276.7°C
- 精度: 0.1°C/LSB
- **来源**: 
  - Temp_D: motorProValue.board_temp (MOS 功率管 / 驱动 PCB)
  - Temp_M: motorProValue.motor_temp (电机绕组 / 外壳)

## 实测数据示例

```
ID=0x101  DLC=48 (33 字节有效)
D[0..2]  = 0x12 0x34 0x56    → Pact = 0x563412 = 5649426 → -6.73 rad (假设 PosMin=-7)
D[3..4]  = 0xAB 0xCD          → Vact = 0xCDAB = 52651 → 12.1 rad/s
D[5..6]  = 0x78 0x9A          → Tact = 0x9A78 = 39544 → 102.3 N·m
...
D[20..21]= 0x10 0x27          → iqref = 0x2710 = 10000 → (10000-10000)/100 = 0 A
D[22..23]= 0x2C 0x01          → iqfdb = 0x012C = 300 → (300-10000)/100 = -97 A (错误，应 >0)
D[24..25]= 0x44 0x4C          → Irms = 0x4C44 = 19524 → (19524-10000)/100 = 95.24 A
D[28]    = 0x30               → Vdc = 48V
D[29..30]= 0x2C 0x01          → Temp_D = 0x012C = 300 → 30.0°C
D[31..32]= 0xF4 0x01          → Temp_M = 0x01F4 = 500 → 50.0°C
```

## 与 0x7FE 扩展状态帧的区别

| 特性 | 0x100+ID (33B) | 0x7FE (16B) |
|------|----------------|-------------|
| 位置字段 | Pact[24] + Pcmd[24] (双通道) | 单通道 int32 0.001° |
| 速度字段 | Vact[16] + Vcmd[16] (双通道) | 单通道 int16 0.1 rpm |
| 转矩字段 | Tact[16] + Tcmd[16] + MIT_T[16] | 无 |
| 电流字段 | iqref[16] + iqfdb[16] + Irms[16] | Irms[16] (0.01A) |
| 温度字段 | Temp_D[16] + Temp_M[16] (0.1°C) | Temp_D[16] + Temp_M[16] (0.1°C) |
| 母线电压 | Vdc[8] (1V) | 无 |
| 故障字段 | Err1[16] + Err2[16] + warn[8] + STA[8] | STA[8] |
| 用途 | 完整控制回路观测 | 兼容 motor_h7 参考工程 |

## 变更历史

### V1.7 (2026-06-03)
- **Breaking Change**: 帧长 27B → 33B
- 新增: Vdc[8], Temp_D[16], Temp_M[16], Irms[16]
- 变更: Err2 从 uint8 扩展为 uint16
- 变更: iqref/iqfdb 映射从 int16 0.01A 改为 uint16 (0.01A + 偏置 10000)
- 删除: Ia 相电流字段 (上位机用 Irms 或自行算三相平衡)

### V1.6 (2026-05-29)
- 初始版本 (27B)
- 包含 Pact/Vact/Tact + Pcmd/Vcmd/Tcmd + iqref/iqfdb/Ia + MIT_T

## 相关文档

- `CLAUDE.md` § CAN-FD 调试协议
- `Core/Src/can_wly.c` `pack_status_frame()` 实现
- `Core/Inc/can_wly.h` 协议定义
- 万里扬 FDCAN 通信协议 V1.7 (工程根目录)
