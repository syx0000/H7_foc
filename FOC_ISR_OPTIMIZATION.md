# FOC 主循环耗时分析与优化方案

## 一、现象数据

```
Inner:178.90 Outer:315.10 Sta:0x00 | Trig:10001Hz Succ:10001 Skip:0 | Enc(us):last=86 min=44 max=88
  TIM1(us): CC4_in=56.2 CC4_out=61.6 Enc_done=55.0 UP_in=0.0 UP_out=0.0
  ADC ISR: entry=7.0us exit=50.9us | duration last=43us max(1s)=67us
```

- **ISR entry=7us**：从硬件事件到 ISR 第一行代码就已经 7us，480MHz M7 上明显偏高 —— 强烈指向 **Cache 未开启**。
- **duration 43~67us**：10kHz 周期 100us，FOC 中断吃掉近 2/3 CPU。
- **last 在 43/60 间跳变**：43us 是普通周期，60us 是叠加了位置/速度环分频的拍。
- **Enc(us) 45~85us**：DPT 编码器帧耗时，异步 DMA，不计入 ADC ISR，但会挤总线、引入抖动。

---

## 二、配置层面的三个大坑（收益最高）

### ① CPU Cache 未启用 ⭐⭐⭐⭐⭐

`Core/Src/main.c:82~95`：只调用了 `MPU_Config()` 和 `HAL_Init()`，**没有** `SCB_EnableICache/DCache`。

- 代码运行在 Flash，不开 I-Cache 时每次取指都吃 Flash 等待周期（STM32H7 @480MHz 约 4~7 WS）。
- 经验值：**开 I-Cache 后纯代码执行时间降到 1/3~1/5**。
- `ISR entry=7us` 是典型的"Cache 没开"特征。

**推荐修改**：在 `main()` 开头、`HAL_Init()` 之前加：
```c
SCB_EnableICache();
SCB_EnableDCache();   // 若 DMA 缓冲区一致性没处理好，可先只开 I-Cache
```

> DCache 注意：ADC 规则通道 DMA 缓冲 `adc_reg_buffer[2]` 需要放到非 Cacheable 区域，或使用 `SCB_InvalidateDCache_by_Addr()`，否则会读到陈旧值。稳妥起见先只开 ICache。

**预期收益：43us → 15~20us**

---

### ② 编译器优化方向错误 ⭐⭐⭐⭐

`MDK-ARM/cubemx_yxsui.uvprojx`：
```xml
<Optim>4</Optim>     <!-- 4 = -O3 -->
<oTime>0</oTime>     <!-- 0 = 为空间优化 -->
```

Keil AC5 的 `Optim=4` 是 `-O3`，但 `oTime=0` 表示"优先空间"，空间策略会破坏热路径的除法/switch/内联。

**推荐修改**：在 Keil 工程 `Options for Target → C/C++` 中勾选 **Optimize for Time**（等价 `oTime=1`）。

**预期收益：再砍 20~30%**

---

### ③ 关键代码未放置 ITCM/DTCM ⭐⭐⭐

`foc/foc_fast/foc_bsp.h:28-34`：
```c
#define ATTR_RAMFUNC              // 空
#define ATTR_PLACE_AT_FAST_RAM_INIT  // 空
```

sin/cos 表、FOC kernel 函数都在 Flash，即使开 Cache 也有 miss 抖动（`max=67us` 就是 cache miss 尾部）。H7 有 64KB ITCM + 128KB DTCM，足够放所有 FOC 热代码。

**预期收益：消除抖动，max 从 67us 接近 typical**

---

## 三、代码层面的热点（按耗时权重排序）

### 🔥 热点 1：`weak_magn_control` 空转 —— **5~10us**

`foc/foc_fast/foc_current_loop.c:101` 每周期调用 `weak_magn_control(controller)`。即使 `USE_WEAK_MAGN = 0`，外层包裹宏只是包了内部赋值分支，**函数本身仍在执行**：

```c
// foc_current_loop.c L213~246
void weak_magn_control(ControllerStruct* controller){
  float Us_buf[32] = {0};                                 // 栈上 128 字节每次清零
  controller_eyou.Us_raw = sqrt(V_d*V_d + V_q*V_q);       // 标准库 sqrt
  controller_eyou.Us = sliding_avg_filter(Us_buf, 32, ...); // 32 次循环求和
  ...
}
```

更糟：`Us_buf` 是栈变量，下次调用又被清零，`sliding_avg_filter` 的累加完全是废功。

**修改**：把 `weak_magn_control` 的调用用 `#if USE_WEAK_MAGN` 包起来，和 `deadtime_compensation` 一样处理。

---

### 🔥 热点 2：`svpwm_calc` 的海量整数除法 —— **5~8us**

`foc/foc_fast/foc_kernel.c:127~237`：每个 case 内部都是 `(Valpha * PWM_T / UDC)` 级别的常量除法，一个 switch 里有 **10+ 次除法**。

- M7 有单周期 SDIV，但前提是编译器能消除除法；`oTime=0` 下常被编成 `__aeabi_idiv`（100+ 周期）。
- L142~148 判扇区时又算了一遍 `divSQRT_3_2 * Valpha / PWMDIV`，和 case 里完全重复。

**修改**：预先把 `(PWM_T / UDC)` 算成 Q15 乘系数；常量除法（`/PWMDIV=1024`、`/32768`、`/65536`）改为 `>> N`。

**预期收益（与 oTime=1 叠加）：2~4us**

---

### 🔥 热点 3：`phase_current_sample` 的常量除法 —— **1.5us**

`foc/foc_fast/foc_current_loop.c:279~282`：
```c
controller->I_a = (int32_t)(raw - offset) * CURRENT_TRANS_NUMERATOR / CURRENT_TRANS_DENOMINATOR;
// NUMERATOR=33, DENOMINATOR=16  ← 除以 16 应该 >>4
```

除以 16 的常量除法在 `Os` 下未必被优化成位移。再加上 `InvertDirflag` 运行期几乎不变，却每拍都做 2 次分支判断。

**修改**：把 `CURRENT_TRANS_DENOMINATOR` 替换为右移宏；`InvertDirflag` 切换时预置函数指针或 phase-map 查表。

---

### 🔥 热点 4：`check_phases_overcurrent_timesliced` 的 50 点循环 —— max 抖动主凶

`foc/foc_fast/foc_current_loop.c:391~442` 的 `process_single_phase`：当 `sample_index>=50` 时进入 **48 迭代 float 比较 + 20 迭代 RMS 比较**的两层循环。每 4 拍触发一次，触发那一拍 ISR 延迟飙到 10us+ —— 对应 **max=67us** 主要来源。

**修改**：
- 改用轻量 peak-hold 算法；
- 或下放到 1kHz 慢环（FOC 10kHz 环根本不需要做过流 RMS）。

---

### 🔥 热点 5：`Encoder_data_Calculate` 的 64 位乘法 —— **~1us**

`foc/foc_fast/encoder_calc.c:110, 195`：
```c
controller->real_position_raw =
    (int32_t)(((uint64_t)temp * 360) >> ENCODER_10BIT_DIV) +
    (controller->circle_count * 360 << 10);
```

`temp * 360` 最大 ~6e9，溢出 uint32，被迫 64 位。

**修改**：改成 `temp * (360 << 10) >> 14` 的等价但只需 32 位的序列（预先合并位移），或直接使用 `__UMULL` intrinsic 读高位。

---

### 🔥 热点 6：PID 函数指针阻断内联 —— **~1us**

`foc/foc_fast/foc_current_loop.c:91,97`：
```c
controller->IncPID_DAxis.PidRun(&controller->IncPID_DAxis);  // 函数指针
controller->IncPID_QAxis.PidRun(&controller->IncPID_QAxis);  // 函数指针
```

M7 间接跳转 + 打破内联 + `IncPIDCal` 内部还有 int64 乘法。

**修改**：电流环直接调用 `IncPIDCal(&...)`，让编译器内联。

---

## 四、推荐修复顺序（按 ROI 降序）

| # | 动作                                                                                   | 类型     | 预计收益             | 风险                                       |
| - | -------------------------------------------------------------------------------------- | -------- | -------------------- | ------------------------------------------ |
| 1 | `main()` 加 `SCB_EnableICache()`（DCache 视 DMA 处理情况再决定）                       | 配置     | **-20us**            | 单独开 ICache 最稳；DCache 需注意 DMA 一致 |
| 2 | Keil 工程勾选 Optimize for Time（`oTime=1`）                                           | 配置     | **-8us**             | 无                                         |
| 3 | `weak_magn_control` 调用处用 `#if USE_WEAK_MAGN` 包起来                                | 代码     | **-5us**             | 无                                         |
| 4 | `check_phases_overcurrent_timesliced` 下放到 1kHz 慢环或换轻量算法                     | 代码     | **-3us（max 降更多）** | 需确认过流保护时序要求                     |
| 5 | PID 函数指针改直接调用 `IncPIDCal`                                                     | 代码     | -1us                 | 无                                         |
| 6 | `svpwm_calc` 预算 `PWM_T/UDC` 系数 + 常量除法改位移                                    | 代码     | -3us                 | 需对比波形                                 |
| 7 | FOC 热代码 / sin 表放 ITCM/DTCM（`ATTR_RAMFUNC` / `ATTR_PLACE_AT_FAST_RAM_INIT`）     | 配置+链接 | 降抖动               | 需改 scatter 文件                          |

---

## 五、立竿见影方案

**只做前 3 项：43us → 预计 10~15us**，留出 70%+ CPU 给控制和通信。

### Step 1：开 I-Cache

`Core/Src/main.c` 的 `main()` 入口：
```c
int main(void)
{
  /* USER CODE BEGIN 1 */
  SCB_EnableICache();
  // SCB_EnableDCache();   // 先不开，待确认 DMA 缓冲一致性
  /* USER CODE END 1 */

  MPU_Config();
  HAL_Init();
  ...
}
```

### Step 2：Keil 工程开 Optimize for Time

`Options for Target → C/C++` → 勾选 `Optimize for Time`。

### Step 3：关掉空转的 weak_magn_control

`foc/foc_fast/foc_current_loop.c:101` 附近：
```c
#if USE_WEAK_MAGN
  weak_magn_control(controller);
#endif
```

---

## 六、验证方法

修改前后对比 `ADC ISR: duration last / max(1s)` 日志字段，关注：
- `last` 是否从 43us 降到 15us 以下；
- `max(1s)` 是否从 67us 显著下降（消除 cache miss / 过流检测触发尾部）；
- `entry` 是否从 7us 降到 1~2us（cache 效果最直接的体现）。

---

## 七、分阶段实施计划

### 阶段 1：低风险高收益（纯代码改动）

| # | 动作 | 改动位置 | 预期收益 | 风险 |
|---|------|---------|---------|------|
| 1 | `main.c` 加 `SCB_EnableICache()`（仅 ICache） | `Core/Src/main.c` USER CODE BEGIN 1 | **-20us** | 零风险（ICache 无 DMA 一致性问题） |
| 2 | Keil 勾选 Optimize for Time | `Options for Target → C/C++` | **-8us** | 零风险 |
| 3 | `weak_magn_control` 加 `#if USE_WEAK_MAGN` | `foc_current_loop.c:101` | **-5us** | 零风险（本就是空转代码） |

**阶段 1 预期：ISR 43us → 10~15us**

### 阶段 2：开启 DCache（需要 MPU 配合，见第八节）

| # | 动作 | 改动位置 | 预期收益 | 风险 |
|---|------|---------|---------|------|
| 4 | CubeMX 配置 MPU Region 1 为非 Cacheable | CubeMX → CORTEX_M7 → MPU Settings | - | 配置错会 HardFault |
| 5 | DMA 缓冲区加 `__attribute__((section(".dma_buffer")))` | adc.c / encoder.c | - | 需改 scatter 文件加段 |
| 6 | `main.c` 加 `SCB_EnableDCache()` | main.c | **-3~5us** | 需先完成 4/5 |
| 7 | `check_phases_overcurrent_timesliced` 下放到 1kHz 慢环 | `foc_current_loop.c:391~442` | **max -10us** | 需确认过流保护时序 |

**阶段 2 预期：ISR 稳定 8~10us，max 抖动大幅降低**

### 阶段 3：ITCM/DTCM 热代码放置（需改链接脚本）

| # | 动作 | 改动位置 | 预期收益 | 风险 |
|---|------|---------|---------|------|
| 8 | 修改 `.sct` scatter 文件，加 ITCM/DTCM 段 | `MDK-ARM/cubemx_yxsui/cubemx_yxsui.sct` | - | 链接脚本改错会无法启动 |
| 9 | `ATTR_RAMFUNC` 宏定义为段属性，sin 表和 FOC kernel 加宏 | `foc_bsp.h` + 各 foc 热函数 | **抖动消除** | 需验证启动流程 |
| 10 | PID 函数指针改直接调用 `IncPIDCal` | `foc_current_loop.c:91,97` | **-1us** | 零风险 |
| 11 | `svpwm_calc` 预算 `PWM_T/UDC` 系数 + 常量除法改位移 | `foc_kernel.c:127~237` | **-3us** | 需对比波形 |

**阶段 3 预期：ISR 5~8us，无抖动**

---

## 八、方案 A 详细设计：MPU 配置非 Cacheable 区 + DMA 缓冲区重定位

### 8.1 背景

开启 DCache 后，CPU 通过 Cache 访问 SRAM，而 DMA 外设直接访问 SRAM 物理内存。两者不同步会导致：

- **DMA → CPU**（ADC/编码器）：DMA 把新数据写进 SRAM，CPU 从 Cache 读到旧值
- **CPU → DMA**（USART TX）：CPU 写进 Cache 但未回写 SRAM，DMA 读到旧值

### 8.2 涉及的 DMA 缓冲区清单

| 缓冲 | 文件位置 | 方向 | 当前所在内存 |
|------|---------|------|-------------|
| `adc_reg_buffer[2]` | `Core/Src/adc.c:38` | DMA → CPU | AXI SRAM (默认 `.bss`) |
| DPT 编码器 RX 缓冲 | `Core/Src/encoder.c` | DMA → CPU | AXI SRAM |
| DPT 编码器 TX 缓冲 | `Core/Src/encoder.c` | CPU → DMA | AXI SRAM |
| USART1 TX 缓冲（printf） | `Core/Src/usart.c` | CPU → DMA | AXI SRAM |
| USART1 RX 缓冲（调试命令） | `Core/Src/usart.c` | DMA → CPU | AXI SRAM |

### 8.3 内存布局设计

STM32H743 有多块 SRAM：
- **D1 AXI SRAM** `0x24000000` 512KB — 默认 `.bss/.data`
- **D2 SRAM1** `0x30000000` 128KB — 适合做 DMA 缓冲（与 D2 外设同域）
- **D2 SRAM2** `0x30020000` 128KB
- **D2 SRAM3** `0x30040000` 32KB
- **D3 SRAM4** `0x38000000` 64KB — 低功耗域，BDMA 用

**选 D2 SRAM1 `0x30000000` 作为 DMA 专用区**，原因：
- ADC/USART/DMA1/DMA2 都在 D2 域，总线路径短
- 独立于主 AXI SRAM，不影响正常变量
- 128KB 足够

### 8.4 MPU Region 1 配置（CubeMX）

在 CubeMX → CORTEX_M7 → MPU Settings 中：

```
Cortex Memory Protection Unit Region 1 Settings:
  MPU Region:              Enabled
  MPU Region Base Address: 0x30000000
  MPU Region Size:         32KB (够用即可，16~64KB 均可)
  MPU SubRegion Disable:   0x0
  MPU TEX field level:     level 1
  MPU Access Permission:   FULL ACCESS
  MPU Instruction Access:  DISABLE
  MPU Shareability Permission: ENABLE
  MPU Cacheable Permission:    DISABLE   ← 关键
  MPU Bufferable Permission:   ENABLE
```

**关键字段解释**：
- `TEX=1, C=0, B=1, S=1` → Normal memory, Non-cacheable, Shareable（ARM MPU 标准配置）
- `Shareable=ENABLE` 保证多总线主（CPU+DMA）看到一致顺序
- `Cacheable=DISABLE` → 绕开 DCache

### 8.5 链接脚本（scatter）修改

`MDK-ARM/cubemx_yxsui/cubemx_yxsui.sct` 当前可能只有默认段，需加 D2 SRAM 段：

```scatter
; 原有部分保持不变
LR_IROM1 0x08000000 0x00200000  {
  ER_IROM1 0x08000000 0x00200000  {
   *.o (RESET, +First)
   *(InRoot$$Sections)
   .ANY (+RO)
   .ANY (+XO)
  }
  RW_IRAM1 0x24000000 0x00080000  {  ; AXI SRAM
   .ANY (+RW +ZI)
  }
}

; 新增：D2 SRAM1 作为非 Cacheable DMA 区
LR_DMA_BUFFER 0x30000000 0x00008000  {
  RW_DMA_BUFFER 0x30000000 0x00008000  {
    .ANY (.dma_buffer)           ; 吸收所有 .dma_buffer 段
  }
}
```

### 8.6 DMA 缓冲区迁移

#### Keil AC5 语法

```c
// adc.c:38
__attribute__((section(".dma_buffer"), aligned(32)))
static uint32_t adc_reg_buffer[2];
```

- `section(".dma_buffer")` 指定段名，与 scatter 对应
- `aligned(32)` 对齐到 Cache Line（虽然非 Cacheable 不严格要求，但相邻变量若在 Cacheable 区可避免伪共享）

#### 需要迁移的变量清单

| 文件 | 变量 | 修改 |
|------|------|------|
| `Core/Src/adc.c` | `adc_reg_buffer[2]` | 加 section 属性 |
| `Core/Src/encoder.c` | DPT RX/TX buffer | 加 section 属性 |
| `Core/Src/usart.c` | `debug_rx_buffer` / TX 临时缓冲 | 加 section 属性 |

### 8.7 启用顺序（main.c）

```c
int main(void)
{
  /* USER CODE BEGIN 1 */
  SCB_EnableICache();
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();              // 先配 MPU（CubeMX 生成，包含 Region 1）

  /* USER CODE BEGIN SysInit */
  SCB_EnableDCache();        // MPU 生效后再开 DCache
  /* USER CODE END SysInit */
  ...
}
```

**顺序关键**：
1. ICache 任何时候都能开
2. DCache 必须在 MPU 配置好之后再开
3. 否则 DMA 缓冲区被 Cache 污染，首帧就错

### 8.8 验证清单

- [ ] `printf` 输出正常（USART TX DMA 方向）
- [ ] DPT 编码器 `real_position` 读数正确（RS485 RX DMA 方向）
- [ ] `Trig/Succ` 计数正常递增（编码器触发 + 应答）
- [ ] `adc_reg_buffer` 读出的 VDC/温度值合理（ADC RX DMA 方向）
- [ ] 调试命令 `logid 100` 可响应（USART RX DMA 方向）
- [ ] ISR `last/max` 下降到预期区间

### 8.9 回退方案

若开 DCache 后出现异常，临时回退两种选择：

1. **代码回退**：注释 `SCB_EnableDCache()`，只保留 ICache —— 损失 -3~5us，但阶段 1 收益不丢
2. **手动同步**：不动 MPU，在 DMA 接收完成回调里手动 `SCB_InvalidateDCache_by_Addr()`，缓冲发送前 `SCB_CleanDCache_by_Addr()`

```c
// 示例：ADC 规则通道 DMA 完成回调
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  SCB_InvalidateDCache_by_Addr((uint32_t*)adc_reg_buffer, sizeof(adc_reg_buffer));
  // ... 正常处理
}
```

手动同步方案简单但每处都要写，容易漏，长期维护成本高。方案 A（MPU + 专用段）是工业惯例。

---

## 九、参考

- STM32H7 Cache 使用注意：AN4839
- Cortex-M7 取指流水线与 Flash ART：AN4891
- Keil AC5 优化开关：ARM Compiler 5 User Guide §4.1
