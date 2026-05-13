# 故障保护系统设计文档

## 架构概览

```
main.c while(1) 1ms tick
  ├── adc_convert()                    ADC → 工程量（VDC/温度）
  ├── dcVoltageProFunc()               母线过/欠压
  ├── boradTempProFunc()               板温过热
  ├── busOverCurrentCheck()            母线过流（仅运行时）
  ├── LockedRotorProFunc()             堵转（仅运行时）
  ├── driverChipFaultCheck()           DRV8353 nFAULT（未接入）
  ├── motorSpeedOverCheck()            过速（仅运行时）
  ├── motorSpeedOffsetCheck()          速度跟随偏差（仅运行时）
  ├── motorPosOffsetCheck()            位置跟随偏差（仅运行时）
  ├── motorCurrentOffsetCheck()        电流跟随偏差（暂未实现）
  ├── motorOverPosCheck()              软位置限位（仅运行时）
  └── CheckAndHandleAllFaultBits()     故障总分发 → 停机

PWM ISR (10kHz)
  └── check_phases_overcurrent_timesliced()  三相电流 RMS 超限

can_wly_tick_1ms()
  └── CAN 超时计数器递减 → CommunicateErr

上电初始化（一次性）
  └── ElecAngleEstimate()              堵转/线序错误检测
```

## 故障检测策略

### 始终检测（不区分运行状态）

| 保护项 | 函数 | 判据 | 说明 |
|--------|------|------|------|
| 母线过压 | `dcVoltageProFunc` | Udc > OverUdc | 保护硬件 |
| 母线欠压 | `dcVoltageProFunc` | Udc < LowUdc | 保护硬件 |
| 板温过热 | `boradTempProFunc` | T > TemBorad | 保护硬件 |
| DRV nFAULT | `driverChipFaultCheck` | GPIO 低电平 | 未接入，保留 |
| 三相电流 RMS | ISR 内 | RMS > UVWCurrentLimit | 10kHz 实时检测 |
| CAN 超时 | `can_wly_tick_1ms` | 200ms 无帧 | 首帧后启用 |

### 仅运行时检测（foc_run == 2）

| 保护项 | 函数 | 判据 | 不运行时行为 |
|--------|------|------|-------------|
| 过流 | `busOverCurrentCheck` | \|I_q\| > OverCurrent | 跳过 + 清零计数 |
| 堵转 | `LockedRotorProFunc` | 大电流 + 低速 | 跳过 + 清零计数 |
| 过速 | `motorSpeedOverCheck` | \|v\| > velocity_Limit | 跳过 |
| 速度偏差 | `motorSpeedOffsetCheck` | 偏差 > 阈值 持续 500ms | 跳过 + 清零计数 |
| 位置偏差 | `motorPosOffsetCheck` | 偏差 > PositionErr | 跳过 + 清零计数 |
| 软位置限位 | `motorOverPosCheck` | pos 超 Flash 限位 | 跳过 |

## 阈值配置（对齐 motor_h7）

### 母线电压

| 参数 | 字段 | 值 | 实际阈值 | 滤波 |
|------|------|-----|---------|------|
| 过压 | `Threshold.OverUdc` | 600 | 60.0V | 10ms |
| 欠压 | `Threshold.LowUdc` | 240 | 24.0V | 10ms |

VDC 换算：`Udc_01V = raw * 33 * 21 / 65535`（分压比 21:1）

### 温度

| 参数 | 字段 | 值 | 实际阈值 | 滤波 |
|------|------|-----|---------|------|
| 板温停机 | `Threshold.TemBorad` | 100 | 100°C | 10ms |
| 板温警告 | `Threshold.TemBoradWarn` | 90 | 90°C | 未使用 |

NTC 参数：B=3950, R0=10k@25°C, 分压 10k（motor_h7 MOS 侧）

### 过流

| 参数 | 字段 | 值 | 实际阈值 | 滤波 |
|------|------|-----|---------|------|
| 过流 | `Threshold.OverCurrent` | 59392 | 58A (Q10) | 10ms |
| 滤波时间 | `Threshold.OverCurrentTime` | 10 | 10ms | - |

### 堵转

| 参数 | 字段 | 值 | 实际阈值 | 滤波 |
|------|------|-----|---------|------|
| 堵转电流 | `Threshold.BlockTorque` | 13312 | 13A (Q10) | - |
| 堵转速度 | `Threshold.BlockSpeed` | 103424 | 1 rpm 电机端 | - |
| 堵转时间 | `Threshold.BlockTime` | 30 | 30ms | - |

判据：`|I_q| > BlockTorque` 且 `|dtheta_mech| < BlockSpeed` 持续 30ms

### 过速

| 参数 | 字段 | 值 | 实际阈值 |
|------|------|-----|---------|
| 速度上限 | `Threshold.velocity_Limit` | 4136960 | 161.6 rpm 载端 |

单次超限即触发（无滤波）

### 速度跟随偏差

| 参数 | 值 | 说明 |
|------|-----|------|
| 阈值 | max(v_ref×10%, 256000) | 10% 或 10rpm 载端死区 |
| 滤波时间 | 500ms | 覆盖斜坡加速时间 |

### 位置跟随偏差

| 参数 | 字段 | 值 | 实际阈值 |
|------|------|-----|---------|
| 位置偏差 | `Threshold.PositionErr` | 40960 | 40° 输出端 |
| 滤波时间 | `DEFAULT_POS_CHECKK_TIME` | 48 | 48ms |

仅位置模式（controller_mode == 1）下生效

### 软位置限位

| 参数 | 来源 | 说明 |
|------|------|------|
| MaxPositionLimit | Flash | 需 PositionLimitFlag==50 |
| MinPositionLimit | Flash | 需 PositionLimitFlag==50 |

### CAN 超时

| 参数 | 值 | 说明 |
|------|-----|------|
| 超时阈值 | 200ms | 首次收到 CAN 帧后启用 |

### 三相电流 RMS

| 参数 | 字段 | 值 | 实际阈值 |
|------|------|-----|---------|
| UVW 电流限 | `Threshold.UVWCurrentLimit` | 6860 | 6.7A (Q10) |

在 PWM ISR 中每 4 拍检测一次，连续 20 次超限触发

## 故障停机动作

`CheckAndHandleAllFaultBits()` 检测到 `ServoErrFlag.All_Flag != 0` 时：

1. `foc_run = 0`
2. `TIM1->CCER &= ~0x0555u`（关闭 PWM 输出）
3. 清指令：`velocity_ref = 0`, `position_ref = 0`, `I_q_ref = 0`, `I_d_ref = 0`
4. 清斜坡：`SpeedSmooth.NowVelocityRef = 0`
5. `ResetControlData()`（重置三环 PID 累积）
6. `clear_all_fault_counters()`（清零所有检测计数器）
7. 打印故障类型 + 关键数据

仅在 `foc_run == 2` 时执行停机动作。不运行时只设标志，不动作。

## 故障复位

### 串口命令
- `logid 163`：调用 `ClearFaults(1)`

### CAN 命令
- `can_wly.c` 控制帧中的故障复位字段

### ClearFaults 动作
1. `ServoErrFlag.All_Flag = 0`
2. `clear_all_fault_counters()`（清零所有检测计数器）
3. 打印 "All faults cleared, ready to restart"

复位后需要重新发 `Runcmd` 启动电机。

## 故障码定义（ServoErrFlag 位域）

| Bit | 字段名 | 故障类型 | 打印标识 |
|-----|--------|---------|---------|
| 0 | OverBusVolErr | 母线过压 | [1] OverBusVol |
| 1 | LowBusVolErr | 母线欠压 | [2] LowBusVol |
| 2 | OverBusCurrentErr | 母线过流 | [3] OverBusCur |
| 3 | HighBoardTempErr | 板温过热 | [4] HighBoardTemp |
| 4 | HighMotorTempErr | 电机过热 | [5] HighMotorTemp |
| 5 | LockedRotorErr | 堵转 | [6] LockedRotor |
| 6 | EncoderErr | 编码器故障 | [7] EncoderErr |
| 7 | DriverChipNfault | DRV8353 故障 | [8] DriverNfault |
| 8 | sto_activated | STO 触发 | [9] STO |
| 9 | MosFault | MOS 管失效 | [10] MosFault |
| 10 | CommunicateErr | CAN 通讯超时 | [11] CommErr |
| 11 | OverSpeedErr | 过速 | [12] OverSpeed |
| 12 | OverPositionErr | 位置超限 | [13] OverPos |
| 13 | PhaseUVolErr | U 相故障 | [14] PhaseU_Err |
| 14 | PhaseVVolErr | V 相故障 | [15] PhaseV_Err |
| 15 | PhaseWVolErr | W 相故障 | [16] PhaseW_Err |
| 16 | PhaseCurrentSampleErr | 相电流采样故障 | [17] IxSampleErr |
| 17 | CommunicateFlag | 通讯状态异常 | [18] CommFlag |
| 18 | DCBusSampleErr | 母线电压采样异常 | [19] UdcSampleErr |
| 19 | BoradTemSampleErr | 板温采样异常 | [20] BoardTSampleErr |
| 20 | MotorTemSampleErr | 电机温度采样异常 | [21] MotorTSampleErr |
| 21 | PhaseOrderErr | 三相线序错误 | [22] PhaseOrderErr |
| 22 | UserCommendValueErr | 用户指令错误 | [23] UserCmdErr |
| 23 | MotorMaxAccErr | 加速度过大 | [24] MaxAccErr |
| 24 | MotorMaxJerkErr | 加加速度过大 | [25] MaxJerkErr |
| 25 | speedOffsetErr | 速度跟随偏差 | [26] SpeedOffset |
| 26 | eepromDataErr | EEPROM 参数异常 | [27] EepromDataErr |
| 27 | posOffsetErr | 位置跟随偏差 | [28] PosOffset |
| 28 | zeroPointErr | 零点故障 | [29] ZeroPointErr |
| 29 | currentOffsetErr | 电流偏差 | [30] CurOffsetErr |
| 30 | flashReadErr | Flash 读取故障 | [31] FlashReadErr |

## 串口故障输出示例

```
========== FAULT DETECTED ==========
ServoErrFlag = 0x00000001 (prev: 0x00000000)
  [1]  OverBusVol    (Udc=620/10V)
PWM disabled, foc_run = 0, run data cleared
====================================
```

## 关键文件

| 文件 | 职责 |
|------|------|
| `foc/foc_app/ifly_fault.c` | 故障检测实现 + 停机动作 |
| `foc/foc_app/ifly_fault.h` | 接口声明 + 阈值宏 + NTC 查表 + 故障码 |
| `foc/foc_app/ifly_fault_api.c` | Threshold 结构体初始化 + setter API |
| `foc/foc_app/ifly_fault_api.h` | Threshold 结构体定义 + 默认值宏 |
| `Core/Src/main.c:232-253` | 1ms 周期任务调度 |
| `Core/Src/can_wly.c:438-447` | CAN 超时检测 |
| `foc/foc_fast/foc_current_loop.c:420-443` | 三相电流 RMS 检测（ISR） |

## 待完善

- [ ] DRV8353 nFAULT GPIO 接入（硬件确认后实现）
- [ ] 电机温度保护（当前只有板温，电机 NTC B=3435 需单独处理）
- [ ] 电流 RMS 过载保护（持续过载 vs 瞬时过流）
- [ ] 故障历史记录（Flash 存储最近 N 次故障码 + 时间戳）
- [ ] CAN 故障上报帧（当前仅串口打印）

---

**最后更新**: 2026-05-13
**维护者**: syx0000
