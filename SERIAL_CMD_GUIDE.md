# 串口调试指令使用指南

## 连接配置

- **串口**: USART1
- **波特率**: 921600
- **数据位**: 8, 无校验, 1 停止位
- **换行**: 发送时无需换行符（DMA IDLE 接收，帧间隔自动分帧）
- **工具**: VOFA+、SSCOM、PuTTY 等均可

---

## 运行控制

### Runcmd - 启动/停止电机

格式: `RuncmdXMYtarZ`

| 参数 | 含义 | 取值 |
|------|------|------|
| X | foc_run | 0=停止, 2=闭环运行 |
| Y | 控制模式 | 1=位置, 3=速度, 4=力矩, 8=CSP, 9=CSV, 10=CST |
| Z | 目标值 | 位置(°), 速度(rpm 输出端), 力矩(Q10 电流) |

**示例**:
```
Runcmd2M3tar20      速度模式，目标 20 rpm（输出端）
Runcmd2M4tar1024    力矩模式，目标 1A（Q10: 1024=1A）
Runcmd2M1tar90      位置模式，目标 90°（输出端）
Runcmd0M3tar0       停止电机
```

**单位换算**:
- 速度: tar × 1024 × 25 = 内部 velocity_ref（载端 Q10）
- 位置: tar × 1024 = 内部 position_ref（输出端 1°/1024）
- 力矩: tar 直接写入 I_q_ref（Q10，1024=1A）

### enable - PWM 使能/失能

```
enable1     使能 PWM，进入力矩模式 I_q_ref=0（安全启动）
enable0     失能 PWM，foc_run=0
```

---

## 日志查看

### logid - 切换周期日志

格式: `logid<N>`

| N | 内容 | 输出格式 |
|---|------|---------|
| 10 | 电角度/位置 | `Angle_elec_360: mechpos, theta_elec, pos_out, pos, speed_rpm` |
| 30 | 电压 | `current_get: V_q, V_d` |
| 40 | 电流环 PI | `current_pi: I_q, I_d, V_q, V_d, I_q_ref, I_d_ref, I_q_ref_filt` |
| 50 | 速度 | `speed: v_ref/1024, v_ref_filt/1024, v_mech/1024, v_out/1024, diff` |
| 60 | PWM CCR | `CCR2, CCR3, CCR4` |
| 70 | 相电流 | `I_a, I_b, I_c`（Q10） |
| 90 | ADC 原始值 | `Ia_raw, Ib_raw, Ic_raw` |
| 100 | 位置 | `position: ref(°), actual(°), error(°), mech_offset_out` |
| 110 | ISR 分段耗时 | `adc_isr_us tot/max read/max enc/max pos/max vel/max cur/max` |
| 120 | 开环测试状态 | `OpenLoop: theta, I_a, I_b, I_c, V_d, V_q`（每 1s） |
| 130 | DPT 编码器统计 | `Inner/Outer角度, 触发频率, 成功/跳过, 耗时`（每 1s） |
| 140 | ISR 时序 | `T0=ADC_in, ADC_out, CC4_in, CC4_out, Enc_done`（每 1s） |
| 150 | ADC 采样率 | `Rate, Count, Ia/Ib raw, Offset, TIM1 done`（每 1s） |
| 151 | 电压/温度 | `Udc(0.1V), Tboard(0.1°C), Tmotor(0.1°C), Raw`（每 1s） |
| 160 | 写 Flash | 一次性，写入后自动清零 |
| 161 | 擦 Flash | 一次性，擦除后自动清零 |
| 162 | RAM vs Flash 对比 | 一次性，打印所有 FlashData 字段 |
| 163 | 清除故障 | 一次性，清除 ServoErrFlag + 所有计数器 |

### logfreq - 设置日志打印周期

格式: `logfreq<N>`（单位 ms，默认 100）

```
logfreq50       50ms 打印一次
logfreq1000     1s 打印一次
```

注意: logid 120/130/140/150/151 固定 1s 周期，不受 logfreq 影响。

---

## PID 调参

### CurrentPID - 电流环 PID

格式: `CurrentPIDKp<a>Ki<b>Kd<c>`

```
CurrentPIDKp45Ki4Kd0     设置电流环 Kp=45, Ki=4, Kd=0
```

### SpeedPID - 速度环 PID

格式: `SpeedPIDKp<a>Ki<b>Kd<c>`

```
SpeedPIDKp1500Ki10Kd0    设置速度环 Kp=1500, Ki=10, Kd=0
```

### PositionPID - 位置环 PID

格式: `PositionPIDKp<a>Ki<b>Kd<c>`

```
PositionPIDKp3016Ki9Kd0  设置位置环 Kp=3016, Ki=9, Kd=0
```

**注意**: PID 修改立即生效但不写 Flash，断电丢失。需持久化请用 `logid160`。

---

## 辨识与校准

### Cali - 电角度偏置辨识

```
Cali
```

执行电角度偏置辨识 → 擦 Flash → 写入新偏置。电机会短暂转动。

### bwtest - 带宽测试与辨识链

格式: `bwtest<N>`

| N | 功能 | 依赖 | 说明 |
|---|------|------|------|
| 1 | 电流环带宽测试 | 无 | 10~2500Hz 扫频，打印幅值/相位 |
| 2 | 速度环带宽测试 | 无 | 1~200Hz，偏置 10rpm 注入 2rpm |
| 3 | Rs/Ld/Lq 辨识 | 无 | 辨识后写 Flash |
| 4 | 磁链 ψ_f 辨识 | Rs | 辨识后写 Flash |
| 5 | 惯量 J 辨识 | ψ_f + Ld/Lq | coast-down 法，写 Flash |
| 6 | 电流环 autoTune | Rs/Lq | IMC 方法自整定 Kp/Ki |
| 7 | 速度环 autoTune | J/ψ_f/NPP | IMC 方法自整定 Kp/Ki |
| 8 | 位置环 autoTune | 无 | 经验公式 |
| 9 | 位置环带宽测试 | 无 | 4~100Hz，CSP 模式注入 2° |

**推荐顺序**: `bwtest3` → `bwtest4` → `bwtest5` → `bwtest6` → `bwtest7` → `bwtest8`

---

## 调试工具

### injectV - 固定电压注入

格式: `injectV<mV>`

```
injectV2000     在 theta=0 注入 V_d=2.0V，持续 5s，每 100ms 打印 I_a/I_d
```

用于验证 PWM 输出尺度和电流采样是否正确。

---

## 故障管理

### 查看故障状态

```
logid151        查看 VDC/温度（确认传感器正常）
logid163        清除所有故障（等效 ClearFaults）
```

### 故障触发时的串口输出

```
========== FAULT DETECTED ==========
ServoErrFlag = 0x00000001 (prev: 0x00000000)
  [1]  OverBusVol    (Udc=620/10V)
PWM disabled, foc_run = 0, run data cleared
====================================
```

### 故障恢复流程

1. 排除故障原因（降压/散热/松开堵转等）
2. 发送 `logid163` 清除故障标志
3. 发送 `Runcmd2M3tar0` 或 `enable1` 重新启动

---

## 典型操作流程

### 首次上电验证

```
logid151                    确认 VDC 和温度读数正常
logid130                    确认编码器通讯正常
enable1                     使能 PWM（力矩=0，电机不动）
Runcmd2M3tar5              速度模式 5rpm 慢转验证
Runcmd0M3tar0              停止
```

### 速度环调试

```
Runcmd2M3tar20             目标 20rpm
logid50                     观察速度跟踪
logfreq20                   加快打印
SpeedPIDKp2000Ki15Kd0      在线调参
logid160                    满意后写 Flash
```

### 位置环调试

```
Runcmd2M1tar45             目标 45°
logid100                    观察位置跟踪
PositionPIDKp3000Ki10Kd0   在线调参
Runcmd2M1tar0              回零
logid160                    写 Flash
```

### 全套辨识（新电机）

```
Cali                        电角度校准
bwtest3                     Rs/Ld/Lq 辨识
bwtest4                     磁链辨识
bwtest5                     惯量辨识
bwtest6                     电流环 autoTune
bwtest7                     速度环 autoTune
bwtest8                     位置环 autoTune
logid162                    确认 Flash 数据
```

---

## 控制模式编号

| 编号 | 宏名 | 说明 | tar 单位 |
|------|------|------|---------|
| 1 | PROFILE_POSITION_MODE | 位置模式（梯形规划） | ° |
| 3 | PROFILE_VELOCITY_MOCE | 速度模式（斜坡滤波） | rpm 输出端 |
| 4 | PROFILE_TORQUE_MODE | 力矩模式 | Q10 电流 |
| 8 | CYCLIC_SYNC_POSITION_MODE | CSP 同步位置 | ° |
| 9 | CYCLIC_SYNC_VELOCITY_MODE | CSV 同步速度 | rpm 输出端 |
| 10 | CYCLIC_SYNC_TORQUE_MODE | CST 同步力矩 | Q10 电流 |

---

## 注意事项

1. **命令不需要换行符**，DMA IDLE 接收自动分帧（帧间隔 >1 字节时间即可）
2. **PID 修改不自动保存**，需手动 `logid160` 写 Flash
3. **bwtest 期间电机会运动**，确保机械安全
4. **故障后必须 `logid163` 清除**才能重新启动
5. **logid 120~151 固定 1s 周期**，不受 logfreq 影响
6. **logid 160/161/162/163 是一次性命令**，执行后自动清零 logid

---

**最后更新**: 2026-05-13
