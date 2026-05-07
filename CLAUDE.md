# STM32H743 FOC电机驱动项目

## 项目概述

基于STM32H743的FOC（磁场定向控制）电机驱动系统，集成DPT双磁编码器。

- **MCU**: STM32H743VIT6
- **主频**: 480MHz (SystemCoreClock)
- **编译器**: Keil MDK-ARM v5.21 (ARM Compiler 5.06)
- **HAL库**: STM32H7xx HAL Driver
- **电机**: motor_h7_0426配套，极对数NPP=8，减速比50:1

## 参考工程

本项目移植自以下两个参考工程：

### HPMicro PHU (`C:\Users\syx19\Desktop\src_git\hpm6e00evk_ifly_phu`)
- **用途**: FOC架构蓝本（初始化流程、PID参数链、编码器计算、调试日志、Flash存储）
- **关键文件**:
  - `hpm_apps/apps/foc/software/foc_app/src/user/freertos_app.c` - `Init_func` 参考初始化顺序
  - `hpm_apps/apps/foc/software/foc_app/src/foc/foc_fast/foc_api.c` - `Init_foc` / `FocOpenTest` / 参数辨识
  - `hpm_apps/apps/foc/software/foc_app/src/foc/foc_fast/foc_data.c` - `ResetControlData` / `InitFlashData`
  - `hpm_apps/apps/foc/software/foc_app/src/foc/foc_fast/foc_bsp.c` - `dbg_cmd_set` / `dbg_log_print`
  - `hpm_apps/apps/foc/software/foc_app/src/foc/foc_fast/encoder.c` - `Encoder_data_Calculate` / `Encoder_out_data_Calculate`
  - `hpm_apps/apps/foc/software/foc_app/src/foc/foc_fast/foc_controller.c` - `set_ver_par(id)` PID参数按硬件ID分组
- **移植策略**: 单位沿用PHU定点格式（position=1°/1024，速度=1/1024rpm，theta_elec=0~65536）

### motor_h7_0426 (`C:\Users\syx19\Desktop\src_git\motor_h7_0426`)
- **用途**: STM32H7硬件参数参考（电流/编码器的单位换算）
- **关键配置**:
  - 16位ADC + 10x运放 + 0.0025Ω采样电阻 → 电流公式 `I = raw*3.3/65535/10/0.0025`
  - 24位编码器 cpr=16777216
  - 极对数=8
  - 减速比=50

## 关键设计

### 电流单位（Q10定点）
16位ADC推导系数：`I_Q10 = (raw-offset) * 33 / 16`（见 `foc_bsp.h` `CURRENT_TRANS_NUMERATOR/DENOMINATOR`）

### 编码器单位（24位）
- `ENCODER_BIT = 1<<24 = 16777216`
- `ENCODER_16BIT_DIV = 8`（电角度shift：`(NPP*raw)%2^24 >> 8` → 0~65536）
- `ENCODER_10BIT_DIV = 14`（位置shift：`raw*360 >> 14` → 1°/1024）
- **DPT inner_raw/outer_raw映射**: 电机端=outer_raw（高速），输出端=inner_raw（低速），与motor_h7约定一致

### 初始化流程（参考PHU Init_func）
```
外设MX_*初始化
 → DWT_Init / DPT_Encoder_Init
 → EN_GATE高电平 / TIM1_PWM_Start / ADC_FOC_Start / ADC_Regular_Start
 → ADC_CalibrateOffsets          电流零点校准
 → set_ver_par(100)              设NPP/DEFAULT_MAX_SPEED/INC_PID_*_KP（motor_h7_0426配套）
 → Init_foc(&controller_eyou)    滤波器/斜坡/InitFlashData（读Flash或填默认）/ResetControlData
 → 同步Ia_offset/Ib_offset
 → Encoder_out_data_Reset        输出端编码器零位
 → htim1 CCER |= 0x0555          使能PWM输出
 → g_foc_openloop_enable = 1
 → USART1_DebugRx_Start
```

### FOC主循环（ADC注入回调 10kHz）
```
读raw_a/raw_b
 → 更新g_foc_current统计
 → Encoder_data_Calculate        电机端
 → Encoder_out_data_Calculate    输出端
 → phase_current_sample          独立相电流处理（raw→I_a/I_b/I_c）
 → FocOpenTest                   电角度+SVPWM
```

### PID参数链（PHU风格）
```
set_ver_par → INC_PID_*_KP → InitFlashData写入FlashData → ResetControlData初始化IncPID_*结构体
```

### Flash存储（HAL直写方案）
- 扇区: Bank2 Sector 7, `0x081E0000`, 128KB
- 驱动: `Core/Src/flash_port.c`（`Flash_EraseSector` / `Flash_WriteData` / `Flash_ReadData`）
- FOC层: `WriteRunDataToFlash()` / `ReadDataFromAddress()`
- 触发: 启动时自动读取（版本不匹配则用默认值写回）+ 运行时 `logid 160` 写入 / `logid 161` 擦除

### 调试串口（USART1 @ 115200，HPU兼容命令格式）
- DMA IDLE接收，`foc_bsp.c` 中 `dbg_cmd_set()` 解析
- 支持命令:
  - `logid<N>`: 切换日志类型（10=角度,30=电压,40=电流PI,50=速度,60=CCR,70=相电流,90=原始ADC,100=位置,160=写Flash,161=擦Flash）
  - `logfreq<N>`: 日志打印周期ms
  - `logtest<N>`: 测试模式（预留）
  - `CurrentPIDKp<a>Ki<b>Kd<c>`: 电流环PID
  - `SpeedPIDKp<a>Ki<b>Kd<c>`: 速度环PID
  - `PositionPIDKp<a>Ki<b>Kd<c>`: 位置环PID
  - `RuncmdXMYtarZ`: 启动运行（cmd=foc_run,M=mode,tar=目标值）

## 硬件配置

### 定时器
- **TIM1**: PWM输出（中央对齐模式2，20kHz，死区时间50ns）
  - CH1/2/3: 三相PWM输出（互补输出）
  - CH4: 输出比较模式（TIMING），用于编码器预触发
  - TRGO: 触发ADC注入采样（10kHz）
  - 中断: 更新中断(UIE) + CC4中断

- **TIM2**: 备用PWM定时器（中央对齐模式1）

- **TIM6**: ADC规则通道触发（1kHz）
  - TRGO: 触发VDC/温度采样

### ADC
- **ADC1/ADC2**: 双ADC同步模式
  - 注入通道: 电流采样（Ia, Ib），TIM1 TRGO触发，10kHz
  - 规则通道: VDC/温度采样，TIM6 TRGO触发，1kHz，DMA传输

### 通信接口
- **USART1**: 调试串口（printf重定向）
- **USART2**: RS485半双工，DPT编码器通信（2.5Mbps）
  - 硬件DE模式，无需软件切换方向
- **FDCAN1**: CAN-FD接口

### GPIO
- **EN_GATE**: 驱动使能引脚

## 编码器

**DPT双磁编码器**（RS485接口）
- 分辨率: 24位（内圈+外圈）
- 通信速率: 2.5Mbps
- 触发方式: TIM1 CC4中断触发异步读取
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

### 典型编译结果
```
Program Size: Code=39668 RO-data=1108 RW-data=140 ZI-data=18484
"cubemx_yxsui\cubemx_yxsui.axf" - 0 Error(s), 0 Warning(s).
Build Time Elapsed: 00:00:19
```

## 代码结构

### 核心模块
- **Core/Src/main.c**: 主程序入口
- **Core/Src/tim.c**: 定时器配置与工具函数
- **Core/Src/adc.c**: ADC配置与采样 + FOC主循环（ADC注入回调）
- **Core/Src/encoder.c**: DPT编码器RS485驱动（异步DMA读取）
- **Core/Src/usart.c**: USART1 printf + 调试命令DMA接收
- **Core/Src/flash_port.c**: STM32H743内部Flash读写封装

### FOC算法（foc/目录）
- **foc_fast/**: FOC核心算法
  - `foc_api.c`: FOC API接口（Init_foc/FocOpenTest/MC_Loop_Schedule）
  - `foc_kernel.c`: FOC核心运算（Clarke/Park变换、SVPWM）
  - `foc_current_loop.c`: 电流环控制 + phase_current_sample
  - `foc_speed_loop.c`: 速度环控制
  - `foc_position_loop.c`: 位置环控制
  - `foc_controller.c`: 控制器初始化 + set_ver_par(id) PID参数
  - `foc_data.c`: Flash数据管理（InitFlashData/ResetControlData/WriteRunDataToFlash）
  - `foc_bsp.c`: 板级支持包（pwm_ccr_set/dbg_cmd_set/dbg_log_print）
  - `encoder_calc.c`: 编码器计算（Encoder_data_Calculate/Encoder_out_data_Calculate）
  - `func_filter.c`: 滤波器（滑动均值等）
  - `func_pid.c`: PID控制器
  - `func_subprogram.c`: 斜坡滤波器等子程序

- **foc_app/**: 应用层功能
  - `ifly_fault.c`: 故障检测与保护
  - `ifly_flux_ident.c`: 磁链辨识
  - `ifly_inertia_ident.c`: 转动惯量辨识

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
uint32_t DWT_CyclesToUs(uint32_t);     // 周期转微秒
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

## 中断优先级

- **TIM1_CC_IRQn** (优先级0): 编码器预触发，最高优先级
- **TIM1_UP_IRQn** (优先级1): PWM更新中断
- **ADC_IRQn**: ADC注入转换完成
- **DMA**: ADC规则通道DMA传输

## 调试技巧

### 串口输出
```c
printf("Debug message\r\n");  // 通过USART1输出
```

### 时间测量
```c
uint32_t start = DWT_GetCycles();
// ... 代码 ...
uint32_t elapsed_us = DWT_CyclesToUs(DWT_GetCycles() - start);
```

### TIM1时间戳
```c
extern volatile uint32_t g_tim1_cc4_cnt;      // CC4中断进入时
extern volatile uint32_t g_tim1_update_cnt;   // 更新中断进入时
```

## 注意事项

1. **编译器限制**: 使用ARM Compiler 5，不支持某些GCC扩展属性
   - `ATTR_RAMFUNC` 已定义为空
   - `ATTR_PLACE_AT_FAST_RAM_INIT` 已定义为空

2. **中央对齐模式**: TIM1使用中央对齐模式2，计数器先升后降
   - 使用 `TIM1_GetLinearCnt()` 获取线性化的计数值

3. **ADC触发**: 
   - 注入通道由TIM1 TRGO触发（10kHz）
   - 规则通道由TIM6 TRGO触发（1kHz）

4. **编码器读取**: 由TIM1 CC4中断异步触发，避免阻塞主循环

## Git信息

- **分支**: main
- **用户**: syx0000
- **最近提交**: ADC规则通道DMA采样实现

## 相关文档

- STM32H743数据手册
- DPT编码器数据手册 v0.5
- FOC算法原理文档
