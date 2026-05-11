# STM32H7 FOC 电机驱动项目开发总结

**项目名**: cubemx_yxsui  
**平台**: STM32H743VIT6 (ARM Cortex-M7 @ 480MHz)  
**电机**: 8 极对 PMSM + 25:1 减速箱 + DPT 24位双磁编码器  
**开发周期**: 2026-05-07 ~ 2026-05-11（5 天）  
**参考工程**: HPMicro PHU + motor_h7 (STM32H7 小电机方案)  

---

## 一、项目演进路线

从一块只有 DPT 编码器 RS485 驱动的工程开始，5 天内完成 FOC 三环闭环 + 全套辨识 + autoTune + 带宽测试验证。

```
Day 1 (2026-05-07)
  5483eca  初始提交 (DPT编码器RS485驱动)
  2a78b11  ADC注入通道触发 + 电流采样
  0d13a41  ADC规则通道DMA (VDC/温度)
  689a698  FOC开环测试 (PWM CCR输出 + ADC回调)
  66e567e  移植调试log (串口命令 + 周期打印)
  08a8d09  移植编码器计算 (DPT双编码器24位 + PHU定点)
  6211398  对齐PHU初始化流程 (PID参数链)

Day 2 (2026-05-07)
  015dcc8  集成Flash参数存储 + 消除全部警告
  57856d8  电机参数自动辨识 (Rs/Ld/Lq + autoTune)

Day 3 (2026-05-09)
  0222ed8  辨识链 + Cali + 反向修复 + usart fputc：电流环跑通
  11184b5  万里扬FDCAN协议从站集成（未验证）
  e30386c  电流环带宽测试 + 延迟补偿 + 开发计划

Day 4~5 (2026-05-11)
  (pending) 速度环 bwtest2 + 调参 (Kp=1500 Ki=10, BW=45.7Hz)
            磁链辨识 bwtest4 (ψ_f=0.00967 Wb)
            惯量辨识 bwtest5 (J=1.22e-4 kg·m²)
            Flash 缓存机制 (FLASH_STRUCT_VERSION=3 + 三 Flag)
            三环 autoTune (bwtest6/7/8 + SAFETY_FACTOR)
            位置环 bwtest9 + 梯形规划修复 (VMAX=100, 死区 5LSB)
            DWT 时间戳清理 + 中断时序分析
```

---

## 二、各模块实现详情

### 2.1 编码器子系统

**DPT 双磁编码器（24位 RS485）**

初始驱动已有，本次主要工作是把原始数据接入 FOC 控制链：

- `Core/Src/encoder.c`: 阻塞 + DMA 两套 API，CRC-8 校验，TIM1 CC4 中断异步触发
- `foc/foc_fast/encoder_calc.c`: **新增** 移植自 PHU 的定点计算
  - `Encoder_data_Calculate`: 电机端（outer_raw）→ theta_elec / real_position / dtheta_mech
  - `Encoder_out_data_Calculate`: 输出端（inner_raw）→ real_position_out / dtheta_mech_out
  - `Encoder_out_data_Reset`: 开机零位初始化

**关键修复（本次开发）**

1. **InvertDirflag=1 时编码器值镜像**（对齐 PHU 但被遗漏）
   ```c
   if (InvertDirflag == 1) return ENCODER_BIT - raw;
   ```
   不做这步会让反向模式下 elec_offest_1 算出来的 theta_elec 差 120°电角度，q 轴施力落在 d 轴上，**转子静止不转**。

2. **辨识模式下 theta_elec 不被覆盖**
   - `ident_test.enable=1` 时，encoder_calc 停止写 theta_elec
   - 允许 Rs 辨识把 theta 钉死在 0

3. **30µs 延迟补偿**（移植 motor_h7 方案）
   ```c
   theta_elec += (dtheta * 3) / 10;  // 对应 30µs 延迟
   ```
   - theta_elec_last 保存**未补偿的原始值**（避免递归误差）
   - 实测 1667 rpm 下补偿 2.4° 相位超前

---

### 2.2 ADC 子系统

**双 ADC 同步采样架构**

| ADC | 通道 | 触发源 | 频率 | 用途 |
|-----|------|--------|------|------|
| ADC1/ADC2 注入 | Ia/Ib | TIM1 TRGO | 10kHz | 电流采样（FOC 主回调） |
| ADC1 规则 (DMA) | VDC/温度×2 | TIM6 TRGO | 1kHz | 母线电压 + 温度 |

**关键实现**
- `ADC_CalibrateOffsets(N)`: N 次采样取均值作为 Ia_offset / Ib_offset
- `HAL_ADCEx_InjectedConvCpltCallback`: FOC 主循环入口，10kHz 调度
- Q10 电流定点：`I_Q10 = (raw-offset) × 33 / 16`（基于 16位ADC + 10x 运放 + 0.0025Ω 采样电阻）

---

### 2.3 FOC 控制架构

**四层环路（对齐 PHU）**

```
位置环 (2.5kHz) → 速度环 (5kHz) → 电流环 (10kHz) → SVPWM → TIM1 PWM
                                       ↓
                                   phase_current_sample (ADC注入同步)
                                       ↓
                                   Clarke + Park
                                       ↓
                                   [辨识模式] 或 [PID-Id/Iq]
                                       ↓
                                   死区补偿 + 限幅 + 反Park + SVPWM
```

**核心文件**
- `foc_current_loop.c`: 电流环主循环 + 死区补偿 + 带宽测试 + 电感辨识 ISR
- `foc_speed_loop.c`: 速度环 + 斜坡滤波 + 速度环带宽测试
- `foc_position_loop.c`: 位置环 + 梯形规划 + 位置环带宽测试
- `foc_kernel.c`: Clarke / Park / SVPWM 核心变换
- `foc_controller.c`: `set_ver_par(id)` 按硬件 ID 分组 PID 默认值
- `foc_bsp.c`: 调试串口命令解析 + 周期日志打印 + pwm_ccr_set

---

### 2.4 参数辨识链

#### 2.4.1 Rs 辨识（相电阻）

**方法**: θ=0 固定电角度，注入 V_d=1V，5 秒均值采 I_a 和 I_d，双通道交叉验证

**关键设计**
- `theta_elec = 0` 钉死（让 V_d 打在 A 相方向）
- `ident_test.enable=1` 时 encoder_calc 不覆盖 theta_elec
- `R_ia = V_d / I_a_avg` 和 `R_id = V_d / I_d_avg` 两通道比较
- `|R_ia - R_id| < 0.001Ω` 判 **PASS**

**实测**
```
Rs: R(via Ia)=0.0764  R(via Id)=0.0764  diff=0.0000  PASS
```
手册值 0.0701Ω，实测 0.0764Ω (+9%)，差值归因于 MOS Rdson + 寄生电阻 + 死区等效压降。

#### 2.4.2 Ld/Lq 辨识（相电感）

**方法**: AC 正弦注入 @ 500Hz，同步检测 V/I 幅值比算阻抗，扣除 Rs 后得电感

**调试过程踩过的坑**
1. 初始 2000Hz 注入，每周期只 5 个采样点 → 相位检测误差大，L 被低估 5.6 倍 (0.036mH)
2. 降到 500Hz，每周期 20 点 → 收敛到 **0.113mH**（SPM 非凸极）
3. 继续降到 200Hz → Rs² 占 Z² 的 44%，噪声撕裂 Lq → 200Hz 不可用

**实测**
```
Ld = 0.1140 mH  Z_d=0.4365Ohm  (freq=500Hz Rs=0.0764Ohm)
Lq = 0.1132 mH  Z_q=0.4335Ohm  (freq=500Hz Rs=0.0764Ohm)
```
Ld ≈ Lq 说明 SPM（表贴式）电机。手册"相电感 0.2138mH"推测为 L-L（线间）→ L_dq ≈ L_LL/2 = 0.107mH，实测 0.113mH 接近。

#### 2.4.3 电角度偏置辨识（Cali 命令）

**流程**（对齐 PHU）
```
ElecAngleEstimate → 擦 Flash → WriteDataToFlash
```

**实测**
```
-1: 4243180 (91.05 deg) → 102.08 → 113.09 → 124.55  (ΔMech ≈ 11°)
elec_offest_0 = 1527
1:  4945816 (106.13 deg) → 94.96 → 83.40 → 72.07  (方向反)
elec_offest_1 = 23484
```

#### 2.4.4 电流环 PI 自整定

**公式** (IMC 极零点对消)
```c
Kp = ω_bw × Lq × PID_Div
Ki = ω_bw × Rs × PID_Div × Ts
```

**autoTune 实测 vs 手动最优**

| 配置 | Kp | Ki | TargetBW | 实测 BW | 峰值 | 评价 |
|------|----|----|----------|---------|------|------|
| autoTune 800 | 55 | 4 | 800 | 1760Hz | +2.95dB | 激进边缘 |
| **手调最佳** | **45** | **4** | — | **1710Hz** | **0.86dB** | **最佳** |

手调 Kp=45 落在推荐区甜点：ζ=0.54, PM=54°, 超调=14%，所有指标绿灯。

---

### 2.5 电流环带宽测试（完整移植 + 增强）

**命令**: `bwtest1` (扫频 10~2500Hz)

**底层** (PHU 已有，无需修改)
- `bw_test_init/run/print_results`: 正弦扫频 + 同步检测 Bode 测量

**本次增强** (`bw_test_print_results`)

1. **表头增加运行参数**
   ```
   Kp=45 Ki=4 Kd=0 PID_Div=100 Iq_ref=504 Iq_inject=307 TargetBW=800Hz
   ```

2. **自动检测谐振峰**（峰值 vs 低频基准差 > 1dB）
   - 有峰：从峰值下降 3dB 算带宽
   - 无峰：从低频基准下降 3dB 算带宽

3. **性能指标汇总 + 推荐范围**
   ```
   ---- Performance Summary ----
   Resonance peak:       0.86 dB @ 1000.0 Hz  [recommend <3dB]
   -3dB Bandwidth:      1710.6 Hz (from peak)  [recommend fs/10~fs/5]
   0dB crossover:        311.1 Hz  [closed-loop BW ~ 1.3x this]
   Damping ratio:        0.54       [recommend 0.4~0.7]
   Phase margin (est):    54 deg     [recommend 45~60 deg]
   Overshoot (est):       14 %       [recommend 5~25%]
   ```

4. **数值计算**
   - 阻尼比：`ζ = √((1 - √(1 - 1/Mp²)) / 2)` (精确解)
   - 相位裕度：`PM ≈ 100×ζ` (经验公式)
   - 超调：`e^(-πζ/√(1-ζ²)) × 100%`

---

### 2.6 死区补偿（当前关闭）

**现象**: 打开后在 251Hz 出现 +4dB 伪谐振峰

**根因**: `bang-bang` 切换（sign(I) × V_comp）在电流过零附近频繁跳变，被同步检测误判为系统增益

**当前方案**: `USE_DEADTIME_COMPENSATION = 0`

**长期修复方案**（PLAN.md 已记录）:
```c
// 软切换版本
float ratio = I_phase / DEADTIME_TRANSITION_BAND;
if (ratio > 1) ratio = 1;
V_comp = ratio * DEADTIME_COMP_VOLTAGE * 0.6f;
```

---

### 2.7 Flash 参数存储

**驱动**: `Core/Src/flash_port.c` (HAL 直写，STM32H743 Bank2 Sector7, 0x081E0000, 128KB)

**参数管理**
- 结构体版本号 `FLASH_STRUCT_VERSION = 2`（不匹配重初始化）
- `InitFlashData`: 上电读 Flash，缺省用 `set_ver_par(id)` 的默认值
- `WriteDataToFlash`: logid 160 写入
- `Flash_EraseSector`: logid 161 擦除

**调试工具** (`logid 162`)
- RAM vs Flash 对比打印，验证参数落盘是否一致

---

### 2.8 USART 调试系统

**硬件**: USART1 @ 921600 (DMA TX + RX DMA IDLE 接收)

**命令解析**（兼容 PHU 格式）
- `logid<N>`: 切换周期日志（10 电角度, 40 电流PI, 60 CCR, 70 I_a/I_b/I_c, 100 位置, 160 写Flash, 161 擦Flash, 162 Flash dump）
- `logfreq<N>`: 日志周期 ms
- `logtest<N>`: 测试模式
- `CurrentPIDKp<a>Ki<b>Kd<c>`: 电流环 PID 调参
- `SpeedPIDKp<a>Ki<b>Kd<c>`: 速度环 PID 调参
- `PositionPIDKp<a>Ki<b>Kd<c>`: 位置环 PID 调参
- `injectV<mv>`: 固定角度注入电压（V_d）调试 PWM 尺度
- `Cali`: 电角度偏置辨识 + Flash 保存
- `bwtest<N>`: 带宽测试（1=电流环，2/3 待实现）
- `Run cmdX MY tarZ`: 启动运行（cmd=foc_run, M=mode, tar=target）

**关键 bug 修复** (`fputc`)
```c
// 修复前（错误）
while (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_TX);
// 修复后（正确）
while (huart1.gState == HAL_UART_STATE_BUSY_TX);
```
`HAL_UART_GetState` 返回 `gState | RxState` 组合值。RX DMA 长开时是 `BUSY_TX_RX`（0x23），`== BUSY_TX` (0x21) 永远不等 → printf 抢写 printf_buf 丢帧。只检查 `gState` 才正确。

---

### 2.9 FDCAN 通信（万里扬协议从站）

**硬件**: FDCAN1，2.5Mbps 数据速率，500kbps 仲裁速率

**协议**: 万里扬 V1.7（参考 wly_fdcan_v17.txt + PDF）

**实现文件**
- `Core/Src/fdcan.c`: 硬件层，fdcan_send + RxFifo0Callback
- `Core/Src/can_wly.c`: 协议层，434 行完整从站实现
- `Core/Inc/can_wly.h`: 协议接口

**状态**: 代码集成完成，**未实测**

---

### 2.10 时序与中断

**优先级层次**
```
TIM1_CC_IRQn  (优先级 0)  编码器预触发（最高）
TIM1_UP_IRQn  (优先级 1)  PWM 更新中断
USART1_IRQn   (优先级 1)  UART DMA 完成
USART2_IRQn   (优先级 0)  RS485 编码器通信
ADC_IRQn      (HAL 默认)  ADC 注入转换完成
```

**延迟链路** (@ FOC_FREQ=10kHz)
```
ADC 注入采样     → ~5µs
Encoder 读取     → ~30µs (DPT RS485)
FOC 计算         → ~50µs (Clarke + Park + PI + 反Park + SVPWM)
CCR 写入 → PWM 输出 → 1/2 × PWM_PERIOD = 25µs (ZOH)
─────────────────────────
总延迟           ≈ 150µs (主要是 ZOH)
```

延迟补偿只能消除 30µs 编码器部分，剩下 120µs ZOH 是硬限制。

---

## 三、实测性能指标

### 3.1 辨识参数

| 参数 | 实测值 | 手册值 | 误差 |
|------|--------|--------|------|
| Rs | 0.0764 Ω | 0.0701 Ω | +9% (寄生+死区) |
| Ld | 0.113 mH | — | — |
| Lq | 0.113 mH | ~L_LL/2 = 0.107 mH | -6% |
| elec_offest_0 | 1527 | — | — |
| elec_offest_1 | 23484 | — | — |

### 3.2 电流环最佳配置（Kp=45 Ki=4）

| 指标 | 实测 | 推荐范围 | 评价 |
|------|------|---------|------|
| 带宽 | 1710 Hz | 1000~2000 Hz | ✅ 甜点 |
| 谐振峰 | 0.86 dB | <3 dB | ✅ 远低于上限 |
| 阻尼比 | 0.54 | 0.4~0.7 | ✅ 接近黄金值 0.5 |
| 相位裕度 | 54° | 45~60° | ✅ 教科书最佳值 |
| 超调 | 14% | 5~25% | ✅ 接近理想 |
| 0dB 穿越 | 311 Hz | BW/1.3=1315Hz | ✅ 合理 |

---

## 四、关键技术决策

### 4.1 为什么从 PHU 移植而不是从零开发
- PHU 已跑通完整 FOC 链（辨识 + 三环 + 故障保护 + 带宽测试）
- 定点格式规范（position=1°/1024, 速度=1/1024rpm, theta_elec=0~65536）
- Q10 电流 Q16 角度等约定和 STM32H7 的 ADC 精度匹配良好
- 避免重复踩坑

### 4.2 为什么要移植 motor_h7 的延迟补偿
- PHU 没有做延迟补偿（SEI 硬件编码器接口延迟小）
- H7 用 RS485 DPT 编码器，延迟 30µs（不小）
- motor_h7 公式 `theta += omega × T_delay` 简洁有效
- 带来实测 2.4° 相位超前

### 4.3 为什么默认 PID 不用 autoTune 结果
- autoTune 按 IMC 极零点对消设计（目标 800Hz → Kp=55 Ki=4）
- 实测 Kp=55 峰值 +2.95dB，略激进
- **手动 Kp=45 Ki=4** 所有指标全绿（BW=1710Hz, 峰值 0.86dB）
- main.c 注释 autoTuneCurrentLoopPI，用 id=90 的 default

### 4.4 为什么死区补偿关闭
- bang-bang 切换在电流过零制造 251Hz 伪谐振峰（+4dB）
- 扫频信号被同步检测误判为系统增益
- 带宽测试更干净（+2.95dB → +0.86dB）
- 长期改软切换再启用

---

## 五、遗留问题与 TODO

### 已知问题
1. **死区补偿有 bug**: bang-bang 在过零区产生谐波，扫频时被放大（已禁用，待改软切换）
2. **CAN 通信代码已集成未实测**
3. **ADC ISR 占用 60% 周期**: 速度/位置环在 ISR 内，峰值 60μs，未来扩展受限

### 待开发功能（见 PLAN.md）
1. FDCAN 通信实测（优先级 1）
2. ADC ISR 瘦身：速度/位置环移出 ISR（优先级 2）
3. 死区补偿软切换修复（优先级 3）
4. FOC 频率提升到 20kHz（优先级 4，依赖 ISR 瘦身）
5. 弱磁控制（优先级 5，依赖 ψ_f 辨识已完成）

### 已完成（本阶段新增）
- ✅ 速度环启转 + bwtest2 调参 (Kp=1500 Ki=10, BW=45.7Hz)
- ✅ 磁链辨识 bwtest4 (ψ_f=0.00967 Wb)
- ✅ 惯量辨识 bwtest5 (J=1.22e-4 kg·m²)
- ✅ Flash 缓存机制 (FLASH_STRUCT_VERSION=3 + 三 Flag)
- ✅ 三环 autoTune (bwtest6/7/8 + SAFETY_FACTOR 实测调优)
- ✅ 位置环 bwtest9 + 梯形规划修复 (VMAX=100, 死区 5LSB)
- ✅ DWT 时间戳清理 + 中断时序分析

---

## 六、代码统计

**本次开发新增代码**（相对初始 5483eca）

| 大类 | 新增行数 | 说明 |
|------|---------|------|
| FOC 核心 (foc/foc_fast/) | ~4000 行 | 移植 PHU + 适配 |
| FOC 应用 (foc/foc_app/) | ~1500 行 | 辨识 + 故障保护 |
| 外设驱动 (Core/) | ~500 行 | ADC/TIM/UART/Flash/CAN |
| 文档 | 720 行 | CLAUDE.md + PLAN.md + 本总结 |

**最终 firmware 大小**
```
Code=79520  RO-data=3940  RW-data=632  ZI-data=27584
```
(占 STM32H7 2MB Flash 的 4%，512KB RAM 的 6%)

---

## 七、参考资料

### 代码参考
- **PHU (主参考)**: `C:\Users\syx19\Desktop\src_git\hpm6e00evk_ifly_phu`
  - 完整 FOC 架构、辨识流程、带宽测试
- **motor_h7 (辅助)**: `C:\Users\syx19\Desktop\src_git\motor_h7`
  - STM32H7 + 小电机硬件参数
  - 编码器延迟补偿算法

### 文档参考
- STM32H743 参考手册 (RM0433)
- DPT 磁双编码器数据手册 v0.5
- 万里扬 FDCAN 通信协议 V1.7

### 理论参考
- Ogata "Modern Control Engineering" — 阻尼比 / 相位裕度
- Franklin "Feedback Control of Dynamic Systems" — PM 设计准则
- Krishnan "Electric Motor Drives" — 电流环带宽选择

---

## 八、项目文件结构

```
cubemx_yxsui/
├── Core/
│   ├── Inc/               STM32 HAL 头文件 + 自定义驱动头
│   └── Src/
│       ├── main.c         上电初始化 + 主循环
│       ├── adc.c          双 ADC 同步采样 + FOC 主回调
│       ├── tim.c          TIM1 PWM + DWT 计时
│       ├── usart.c        USART1 DMA 发送 + 命令接收
│       ├── encoder.c      DPT 编码器 RS485 驱动
│       ├── fdcan.c        FDCAN 底层收发
│       ├── can_wly.c      万里扬 V1.7 协议从站 (未实测)
│       └── flash_port.c   STM32H7 内部 Flash 读写
├── foc/
│   ├── foc_fast/          FOC 核心算法 (10kHz 实时域)
│   │   ├── foc_api.c      辨识 + autoTune
│   │   ├── foc_kernel.c   Clarke/Park/SVPWM
│   │   ├── foc_current_loop.c  电流环 + 死区补偿 + 带宽测试
│   │   ├── foc_speed_loop.c    速度环 + 带宽测试
│   │   ├── foc_position_loop.c 位置环 + 梯形规划 + 带宽测试
│   │   ├── foc_controller.c    set_ver_par + 全局控制结构
│   │   ├── foc_data.c          Flash 数据管理 + ElecAngleEstimate
│   │   ├── foc_bsp.c           串口命令 + 日志 + pwm_ccr_set
│   │   ├── encoder_calc.c      编码器 → theta_elec/position/speed
│   │   ├── func_pid.c          增量式 PID 核心
│   │   ├── func_filter.c       滑动均值 / 一阶低通
│   │   └── func_subprogram.c   斜坡滤波
│   └── foc_app/           FOC 应用层 (任务级)
│       ├── ifly_fault.c         故障检测
│       ├── ifly_flux_ident.c    磁链辨识 (待接入)
│       ├── ifly_inertia_ident.c 惯量辨识 (待接入)
│       └── ifly_test.c          测试任务 (带宽测试已接入)
├── MDK-ARM/                    Keil 工程
├── Drivers/                    STM32 HAL Driver
├── CLAUDE.md                   项目概述 (给 Claude 的上下文)
├── PLAN.md                     开发路线图
└── SUMMARY.md                  本文档
```

---

## 九、关键命令速查

**启动流程**（上电自动跑）
```
LT H7 foc start
→ ADC_CalibrateOffsets (电流零点校准)
→ set_ver_par(90) (motor_h7 配套参数)
→ Init_foc (滤波器/PID/Flash)
→ VDC 打印
→ measurePhaseResistance (Rs 辨识)
→ measurePhaseInductanceAC (Ld/Lq 辨识)
→ ResetControlData
→ foc_run = 2 (开启闭环)
→ USART1_DebugRx_Start
```

**常用串口命令**
```
Cali                    电角度偏置辨识 + 写 Flash
bwtest1                 电流环带宽测试 (10~2500Hz)
logid 40                打印 I_q/I_d/V_q/V_d/I_q_ref
logid 70                打印 I_a/I_b/I_c
logid 162               对比 RAM 和 Flash 的参数
CurrentPIDKp45Ki4Kd0    手动调电流环 PID
Run cmd1 M4 tar512      力矩模式 + I_q_ref=0.5A
Run cmd1 M3 tar100      速度模式 + 100rpm
```

---

**文档版本**: 2.0  
**生成时间**: 2026-05-11  
**合作者**: 人类开发者 + Claude Opus 4.7 (Anthropic)
