# STM32H743 FOC电机驱动项目

## 项目概述

基于STM32H743的FOC（磁场定向控制）电机驱动系统，集成DPT双磁编码器。

- **MCU**: STM32H743VIT6
- **主频**: 480MHz (SystemCoreClock)
- **编译器**: Keil MDK-ARM v5.21 (ARM Compiler 5.06)
- **HAL库**: STM32H7xx HAL Driver
- **电机**: motor_h7_0426配套，极对数NPP=8，减速比25:1

## 参考工程

本项目移植自以下两个参考工程：

### HPMicro PHU (`C:\Users\syx19\Desktop\src_git\hpm6e00evk_ifly_phu`)
- **用途**: FOC架构蓝本（初始化流程、PID参数链、编码器计算、调试日志、Flash存储）
- **移植策略**: 单位沿用PHU定点格式（position=1°/1024，速度=1/1024rpm，theta_elec=0~65536）

### motor_h7_0426 (`C:\Users\syx19\Desktop\src_git\90_product_260424`)
- **用途**: STM32H7硬件参数参考（电流/编码器的单位换算）
- **关键配置**:
  - 16位ADC + 10x运放 + 0.0025Ω采样电阻 → 电流公式 `I = raw*3.3/65535/10/0.0025`
  - 24位编码器 cpr=16777216
  - 极对数=8
  - 减速比=25

## 关键设计

### 电流单位（Q10定点）
16位ADC推导系数：`I_Q10 = (raw-offset) * 33 / 16`（见 `foc_bsp.h` `CURRENT_TRANS_NUMERATOR/DENOMINATOR`）

### 编码器单位（24位）
- `ENCODER_BIT = 1<<24 = 16777216`
- `ENCODER_16BIT_DIV = 8`（电角度shift：`(NPP*raw)%2^24 >> 8` → 0~65536）
- `ENCODER_10BIT_DIV = 14`（位置shift：`raw*360 >> 14` → 1°/1024）
- **DPT inner_raw/outer_raw映射**: 电机端=outer_raw（高速），输出端=inner_raw（低速）

### 单位约定
- **位置** (`real_position_out` / `position_ref`): 输出端 1°/1024 LSB
- **速度** (`velocity_ref` / `velocity_ref_filterd`): 载端 rpm × 1024 × 25 (内部含减速比)
- **电机端速度** (`dtheta_mech`): 电机端 rpm × 1024
- **电流** (`I_q` / `I_d` / `I_q_ref`): Q10 A (1024 = 1A)
- **电压** (`V_q` / `V_d`): Q10 V (1024 = 1V)

### 初始化流程（参考PHU Init_func）
```
外设MX_*初始化
 → DWT_Init / DPT_Encoder_Init
 → EN_GATE高电平 / TIM1_PWM_Start / ADC_FOC_Start / ADC_Regular_Start
 → ADC_CalibrateOffsets          电流零点校准
 → set_ver_par(90)              设NPP/DEFAULT_MAX_SPEED/INC_PID_*_KP（motor_h7_0426配套）
 → htim1 CCER |= 0x0555          使能PWM输出
 → Init_foc(&controller_eyou)    滤波器/斜坡/InitFlashData(电角度辨识)/ResetControlData
 → 同步Ia_offset/Ib_offset
 → Encoder_out_data_Reset        输出端编码器零位
 → identifyMotorParamsCached     Flash命中则跳过，否则跑 Rs/Ld/Lq 辨识并写 Flash
 → ResetControlData              清辨识期间积分残留
 → foc_run = 2 闭环使能
 → USART1_DebugRx_Start
 → can_wly_init
```

### FOC主循环（ADC注入回调 10kHz）
```
读raw_a/raw_b
 → 更新g_foc_current统计
 → Encoder_data_Calculate        电机端 (theta_elec / dtheta_mech / real_position)
 → Encoder_out_data_Calculate    输出端 (real_position_out / dtheta_mech_out)
 → phase_current_sample          独立相电流处理（raw→I_a/I_b/I_c）
 → MC_Loop_Schedule:
   - 位置环 (2.5kHz, /POSITION_CALCULATE_DIV=4)
   - 速度环 (5kHz, /VELOCETY_CALCULATE_DIV=2)
   - 电流环 + SVPWM (10kHz)
```

### PID参数链（PHU风格）
```
set_ver_par → INC_PID_*_KP → InitFlashData写入FlashData → ResetControlData初始化IncPID_*结构体
```

### Flash存储（HAL直写方案）
- **扇区**: Bank2 Sector 7, `0x081E0000` (MOTORID0_RUN_DATA_ADDRESS), 128KB
- **驱动**: `Core/Src/flash_port.c`（`Flash_EraseSector` / `Flash_WriteData` / `Flash_ReadData`）
- **FOC层**: `WriteRunDataToFlash()` / `ReadDataFromAddress()`
- **结构体版本**: `FLASH_STRUCT_VERSION = 3`（不匹配时强制重新初始化所有字段 + 跑全部辨识）
- **触发**: 上电自动读 + `bwtest3/4/5` 辨识后写 + `logid 160` 手动写 / `logid 161` 擦除

#### Flash 预留字段映射
| 字段 | 类型 | 用途 | 有效标志 |
|------|------|------|---------|
| `temp1` | int32(float) | Rs (Ω) | `MotorParamFlag` |
| `temp2` | int32(float) | Ld (H) | `MotorParamFlag` |
| `temp3` | int32(float) | Lq (H) | `MotorParamFlag` |
| `temp7` | int32(float) | ψ_f 磁链 (Wb) | `FluxIdentFlag` |
| `temp8` | int32(float) | J 惯量 (kg·m²) | `InertiaIdentFlag` |

Flag 值 == `OFFEST_IS_CORRECTED_FLAG` (50) 表示有效，其它视为无效（0xFFFF 全新 Flash / 0 重置）。

### 调试串口（USART1 @ 921600，HPU兼容命令格式）
- DMA IDLE接收，`foc_bsp.c` 中 `dbg_cmd_set()` 解析
- 主循环 `Test_log_print()` 轮询带宽测试 done 标志，异步打印结果（不阻塞 ISR）

#### 命令列表
- `logid<N>`: 切换周期日志
  - 10=电角度 / 11=输出端编码器调试 / 30=电压 / 40=电流PI / 50=速度 / 60=CCR / 70=相电流 / 90=原始ADC / 100=位置
  - 160=写Flash / 161=擦Flash / 162=对比 RAM vs Flash 参数 / 163=清除故障标志
- `logfreq<N>`: 日志打印周期 ms
- `CurrentPIDKp<a>Ki<b>Kd<c>`: 手动调电流环 PID
- `SpeedPIDKp<a>Ki<b>Kd<c>`: 手动调速度环 PID
- `PositionPIDKp<a>Ki<b>Kd<c>`: 手动调位置环 PID
- `injectV<mv>`: 固定角度注入电压（V_d）调试 PWM 尺度
- `Cali`: 电角度偏置辨识 + Flash 保存
- `RuncmdXMYtarZ`: 启动运行（cmd=foc_run, M=mode, tar=目标值）
- `enable<0/1>`: PWM 使能控制（1=使能，0=失能+停止FOC）
- **bwtest 测试链** (按依赖顺序):
  - `bwtest1`: 电流环带宽测试 (10~2500Hz)
  - `bwtest2`: 速度环带宽测试 (1~200Hz, 偏置 10rpm 注入 2rpm)
  - `bwtest3`: 重新辨识 Rs/Ld/Lq → 写 Flash
  - `bwtest4`: 磁链辨识 ψ_f (依赖 Rs) → 写 Flash
  - `bwtest5`: 惯量辨识 J (依赖 ψ_f + Ld/Lq) → 写 Flash
  - `bwtest6`: 电流环 autoTune (依赖 Rs/Lq)
  - `bwtest7`: 速度环 autoTune (依赖 J/ψ_f/NPP)
  - `bwtest8`: 位置环 autoTune (经验公式)
  - `bwtest9`: 位置环带宽测试 (4~100Hz, CSP 模式注入 2°)

## 实测参数（已落盘 Flash）

| 参数 | 实测值 | 说明 |
|------|--------|------|
| Rs | 0.0764 Ω | 手册 0.0701Ω，+9%（寄生 + 死区） |
| Ld | 0.113 mH | SPM 非凸极 |
| Lq | 0.113 mH | 与 Ld 一致 |
| ψ_f | 0.00967 Wb | 磁链 |
| J | 1.22e-4 kg·m² | 折合电机轴 |
| NPP | 8 | 极对数 |
| 减速比 | 25:1 | 输出端：电机端 |

## 三环 PID 实测最佳值

| 环路 | Kp | Ki | PID_Div | BW | 峰值 | 阻尼比 | PM |
|------|-----|-----|---------|-----|------|--------|-----|
| 电流环 | 45 | 4 | 100 | 1710Hz | +0.86dB | 0.54 | 54° |
| 速度环 | 1500 | 10 | 65000 | 45.7Hz | +1.8dB | - | 67° |
| 位置环 | 3016 | 9 | 100 | 25.6Hz | +0.58dB | - | - |

**autoTune SAFETY_FACTOR**（IMC 理论值打折，补偿实测非理想性）:
- 电流环: 1.0（无折扣）
- 速度环: 0.6（补偿电流环 + 速度滤波 + 减速箱滞后）
- 位置环: 0.4（机械刚度有限 + 减速箱反向间隙）

## 硬件配置

### 定时器
- **TIM1**: PWM输出（中央对齐模式2，20kHz）
  - CH1/2/3: 三相PWM输出（互补输出）
  - CH4: 输出比较模式（TIMING），用于编码器预触发
  - TRGO: 触发ADC注入采样（10kHz）
  - 中断: CC4中断（DPT 异步读取触发）
  - 死区配置: MCU端DTG=12 (50ns)，实际由DRV8353RH内部100ns主导，有效死区100ns

- **TIM2**: 备用PWM定时器（中央对齐模式1）

- **TIM6**: ADC规则通道触发（1kHz）
  - TRGO: 触发VDC/温度采样

- **TIM7**: HAL Timebase（替代 SysTick，1kHz）

### ADC
- **ADC1/ADC2**: 双ADC同步模式
  - 注入通道: 电流采样（Ia, Ib），TIM1 TRGO触发，10kHz
  - 规则通道: VDC/温度采样，TIM6 TRGO触发，1kHz，DMA传输

### 通信接口
- **USART1**: 调试串口 (921600 baud，DMA TX + DMA IDLE RX)
- **USART2**: RS485半双工，DPT编码器通信（2.5Mbps）
  - 硬件DE模式，无需软件切换方向
  - DMA1_Stream0 = TX, DMA1_Stream1 = RX
- **FDCAN1**: CAN-FD 万里扬协议从站

### GPIO
- **EN_GATE**: DRV8353 驱动使能
- **LED_RUN**: CC4 中断指示灯

### 驱动芯片
- **DRV8353RH**: 三相栅极驱动器（硬件接口版）
  - 工作模式: 6×PWM（MODE引脚接GND）
  - 驱动能力: IDRIVEP≈300mA / IDRIVEN≈600mA（IDRIVE引脚470kΩ下拉）
  - 电流采样增益: 10 V/V（GAIN引脚配置）
  - 内部死区: 100ns（硬件固化，不可配置）
  - 与MCU死区叠加: 实际有效死区 = max(MCU_DTG, DRV_internal) = 100ns

## 编码器

**DPT双磁编码器**（RS485接口）
- 分辨率: 24位（内圈+外圈）
- 通信速率: 2.5Mbps
- 触发方式: TIM1 CC4中断触发异步读取
- 读取耗时: ~44μs (硬件传输)
- 驱动文件: `Core/Src/encoder.c`

## 编译方法

### 使用Keil IDE
1. 打开 `MDK-ARM/cubemx_yxsui.uvprojx`
2. 点击 Build 按钮或按 F7
3. 编译产物在 `MDK-ARM/cubemx_yxsui/` 目录

### 使用命令行

**增量编译（build）**：
```bash
"C:/Keil_v5/UV4/UV4.exe" -b "MDK-ARM/cubemx_yxsui.uvprojx" -j0
```

**全量编译（rebuild）**：
```bash
"C:/Keil_v5/UV4/UV4.exe" -r "MDK-ARM/cubemx_yxsui.uvprojx" -j0
```

**编译日志位置**: `MDK-ARM/cubemx_yxsui/cubemx_yxsui.build_log.htm` (HTML格式)

**编译产物**:
- `MDK-ARM/cubemx_yxsui/cubemx_yxsui.axf` - ELF可执行文件
- `MDK-ARM/cubemx_yxsui/cubemx_yxsui.hex` - HEX烧录文件
- `MDK-ARM/cubemx_yxsui/cubemx_yxsui.map` - 链接映射文件

### 当前编译规模
```
Program Size: Code=81132 RO-data=4640 RW-data=708 ZI-data=27588
"cubemx_yxsui\cubemx_yxsui.axf" - 0 Error(s), 0 Warning(s).
```

## 烧录（CMSIS-DAP）

工程已在 `MDK-ARM/cubemx_yxsui.uvoptx` 配置 `BIN\CMSIS_AGDI.dll`，Flash 算法 `STM32H7x_2048.FLM`，目标 Bank1 起始 `0x08000000` / 大小 `0x200000`。

### 命令行烧录
```bash
"C:/Keil_v5/UV4/UV4.exe" -f "MDK-ARM/cubemx_yxsui.uvprojx"
```
- 退出码 0 = 成功；UV4 无 stdout，烧录结果靠退出码判断。
- **坑**: 加 `-o <logfile>` 参数会让 UV4 段错误 (ExitCode=139)，不要加日志重定向。
- 该命令会 Erase → Program → Verify → Reset Run，可直接作为"DAP 重启"使用（抓 log 时用这个代替按复位键）。

## 串口抓 log

- **默认口**: COM4 @ 921600 (USART1, DMA TX)
- **脚本**: `tools/capture_com.ps1` (PowerShell + System.IO.Ports.SerialPort)
- **注意**: 这台机器上 `python` 是 Windows App 别名（静默退出，不可用），所以用 PowerShell 不用 pyserial

```bash
# 后台抓 30s 到 serial_log.txt
powershell -ExecutionPolicy Bypass -File tools/capture_com.ps1 \
    -PortName COM4 -Baud 921600 -Seconds 30 -OutFile serial_log.txt
```

参数: `-PortName` / `-Baud` / `-Seconds` / `-OutFile`。

### 抓开机 log 的正确顺序
1. 后台启动 `capture_com.ps1`（先占住串口）
2. 触发复位：`UV4 -f ...` 或按板子复位键
3. 等抓取超时后 `cat serial_log.txt`

### 正常开机 log 关键行（用于判断启动是否健康）
```
LT H7 foc start
ADC calibration done: Off_a=32677 Off_b=32904     ← ADC 零点应居中 (~32768)
Flash: InvertDirflag=1, mech_offest_out=0, elec_offest_0/1=12723/34402
FlashData: CurPID=45/4/0 SpdPID=1500/10/0 PosPID=3000/9/0 FF=300
VDC ADC raw = 43591, voltage (before divider scaling) = 2.195 V   ← 分压前电压
Motor params loaded from Flash: Rs=0.0794 Ohm  Ld=0.1133 mH  Lq=0.1163 mH
FOC initialization done, NPP=8, foc_run=2         ← 闭环使能成功
```

### 故障诊断
`FAULT! ServoErrFlag=0x<N>, PWM disabled` 的 bit 含义见 `foc/foc_app/ifly_fault.c` 中 `ServoErrFlag.Bit.*` 赋值处（grep `ServoErrFlag.Bit` 即得）。常见：
- `0x02` = LowBusVolErr（母线欠压，通常电源没接或 VDC 通路异常）
- `0x01` = OverBusVolErr, `0x04` = HighBoardTempErr, `0x08` = OverBusCurrentErr, 等。

## 代码结构

### 核心模块
- **Core/Src/main.c**: 主程序入口
- **Core/Src/tim.c**: 定时器配置与工具函数
- **Core/Src/adc.c**: ADC配置与采样 + FOC主循环（ADC注入回调）
- **Core/Src/encoder.c**: DPT编码器RS485驱动（异步DMA读取）
- **Core/Src/usart.c**: USART1 printf + 调试命令DMA接收
- **Core/Src/flash_port.c**: STM32H743内部Flash读写封装
- **Core/Src/can_wly.c**: 万里扬 FDCAN V1.7 协议从站（未实测）
- **Core/Src/stm32h7xx_it.c**: 中断处理 + DWT 时间戳

### FOC算法（foc/目录）
- **foc_fast/**: FOC核心算法
  - `foc_api.c`: Init_foc / 辨识 / autoTune / Cached 加载
  - `foc_kernel.c`: Clarke/Park/SVPWM 核心变换
  - `foc_current_loop.c`: 电流环 + 死区补偿 + 带宽测试 + 电感辨识 ISR
  - `foc_speed_loop.c`: 速度环 + 斜坡滤波 + 速度环带宽测试
  - `foc_position_loop.c`: 位置环 + 梯形规划 + 位置环带宽测试
  - `foc_controller.c`: set_ver_par + 全局控制结构
  - `foc_data.c`: Flash 数据管理 + ElecAngleEstimate
  - `foc_bsp.c`: 串口命令 + 日志 + pwm_ccr_set
  - `encoder_calc.c`: 编码器 → theta_elec / position / speed + 延迟补偿
  - `func_pid.c`: 增量式 PID 核心
  - `func_filter.c`: 滑动均值 / 一阶低通
  - `func_subprogram.c`: 斜坡滤波

- **foc_app/**: FOC 应用层（任务级，阻塞调用）
  - `ifly_fault.c`: 故障检测
  - `ifly_flux_ident.c`: 磁链辨识 (runFluxIdent)
  - `ifly_inertia_ident.c`: 惯量辨识 (runInertiaIdent, coast-down 法)
  - `ifly_test.c`: 测试任务编排（带宽测试 / 辨识入口 / autoTune 入口）

### 跨平台适配

项目从HPMicro平台移植到STM32，已完成以下适配：

1. **类型定义** (`foc/foc_fast/foc_bsp.h`):
   - HPMicro硬件类型 → STM32 stub类型
   - Windows风格类型 (BOOL, UINT16等)
   - 标准宏定义 (M_PI, MIN, MAX等)

2. **RTOS移除**:
   - 注释掉所有 `FreeRTOS.h` 和 `task.h` 包含
   - `vTaskDelay()` → `HAL_Delay()`

3. **EtherCAT移除**:
   - 注释掉 `cia402appl.h` 包含
   - 注释掉 `TCiA402Axis` 相关代码

## 关键函数

### 定时器相关
```c
void DWT_Init(void);                    // 初始化DWT周期计数器
uint32_t DWT_GetCycles(void);           // 获取当前周期数
uint32_t DWT_GetMicros(void);           // 获取微秒数
uint32_t DWT_CyclesToUs(uint32_t);      // 周期转微秒
void TIM1_PWM_Start(void);              // 启动TIM1 PWM和中断
uint32_t TIM1_GetLinearCnt(void);       // 获取TIM1线性计数值
```

### ADC相关
```c
void ADC_FOC_Start(void);               // 启动ADC注入采样（电流）
void ADC_Regular_Start(void);           // 启动ADC规则采样（VDC/温度）
void ADC_CalibrateOffsets(uint32_t);    // 电流零点校准
```

### 编码器相关
```c
void DPT_Encoder_Init(UART_HandleTypeDef*);
DPT_Status DPT_TriggerDualAngleWithStatusRead_Async(void);
void DPT_GetLatestAngles(DPT_Angles*);
uint8_t DPT_HasNewData(void);
```

### FOC 辨识与 autoTune
```c
float measurePhaseResistance(ControllerStruct*);          // Rs 辨识
void measurePhaseInductanceAC(ControllerStruct*, float);  // Ld/Lq 辨识
void identifyMotorParamsCached(ControllerStruct*);        // Flash 命中则跳过, 否则辨识 + 写 Flash
void autoTuneCurrentLoopPI(float Rs, float Ld, float Lq);
void autoTuneSpeedLoopPI(float J, float psi_f, uint8_t pp);
void autoTunePositionLoopPI(void);                        // 无入参, 走经验公式
```

## 中断优先级（NVIC_PRIORITYGROUP_4）

| IRQ | 优先级 | 用途 |
|------|--------|------|
| TIM1_CC_IRQn | 0 | 编码器 CC4 预触发 |
| ADC_IRQn | 0 | FOC 主循环（注入完成回调） |
| USART2_IRQn | 0 | DPT RS485 IDLE/Error |
| DMA1_Stream0/1_IRQn | 0 | USART2 TX/RX DMA |
| DMA1_Stream3/4_IRQn | 0 | USART1 TX/RX DMA |
| DMA2_Stream6_IRQn | 0 | ADC 规则通道 DMA |
| FDCAN1_IT0_IRQn | 6 | CAN 通信 |
| USART1_IRQn | 8 | 调试串口 |
| TIM7_IRQn | 15 | HAL Timebase |

**说明**: 同优先级中断不互相抢占，按 pending 顺序执行。详见 `中断配置清单.md`。

## 调试技巧

### 串口输出
```c
printf("Debug message\r\n");  // 通过USART1输出 (921600)
```

### 时间测量
```c
uint32_t start = DWT_GetCycles();
// ... 代码 ...
uint32_t elapsed_us = DWT_CyclesToUs(DWT_GetCycles() - start);
```

### TIM1 / ADC 时间戳（DWT 周期, 480MHz, 1us=480 ticks）
```c
extern volatile uint32_t g_tim1_cc4_cycles;      // CC4中断进入时
extern volatile uint32_t g_tim1_cc4_exit_cycles; // CC4中断退出时
extern volatile uint32_t g_tim1_enc_done_cycles; // 编码器 DMA 完成 ISR 时
extern volatile uint32_t g_adc_isr_in_cycles;    // ADC ISR 进入时
extern volatile uint32_t g_adc_isr_out_cycles;   // ADC ISR 退出时
extern volatile uint32_t g_adc_isr_cycles;       // ADC ISR 上一次耗时
extern volatile uint32_t g_adc_isr_cycles_max;   // ADC ISR 历史最大耗时
```

### ADC ISR 实测耗时
- **稳态**: ~42μs（占 10kHz 周期 42%）
- **速度环参与拍**: ~58~60μs（FOC 计算压力下）
- **占周期**: 接近 60%，未来扩展功能（弱磁、更高带宽）需把速度/位置环移出 ISR

## 注意事项

1. **编译器限制**: 使用ARM Compiler 5，不支持某些GCC扩展属性
   - `ATTR_RAMFUNC` 已定义为空
   - `ATTR_PLACE_AT_FAST_RAM_INIT` 已定义为空

2. **中央对齐模式**: TIM1使用中央对齐模式2，计数器先升后降
   - 使用 `TIM1_GetLinearCnt()` 获取线性化的计数值

3. **ADC触发**:
   - 注入通道由TIM1 TRGO触发（10kHz）
   - 规则通道由TIM6 TRGO触发（1kHz）

4. **编码器读取**: 由TIM1 CC4中断异步触发，避免阻塞主循环。`g_tim1_enc_done_cycles` 在 DMA 完成 ISR 里更新，**受 ADC ISR 抢占影响会延迟**，不是硬件物理完成时刻。

5. **位置环梯形规划**:
   - `POS_TRAPEZOID_DEFAULT_VMAX_RPM = 100` 输出端 rpm（对齐 MaxSpeed）
   - `POS_TRAPEZOID_DEFAULT_AMAX_RPS = 230` 输出端 rpm/s
   - 已对齐判断带 5 LSB (≈0.005°) 死区，避免 PID 未收敛触发重启

## Git信息

- **分支**: main
- **用户**: syx0000
- **最近主要工作**: 三环 PID autoTune + 全套带宽测试 (bwtest1~9) + Flash 缓存 + 梯形规划修复

## 相关文档

- `SUMMARY.md` - 开发总结
- `PLAN.md` - 开发路线
- `中断配置清单.md` - 中断/DMA 配置详情
- STM32H743 数据手册 (RM0433)
- DPT 编码器数据手册 v0.5
- 万里扬 FDCAN 通信协议 V1.7
