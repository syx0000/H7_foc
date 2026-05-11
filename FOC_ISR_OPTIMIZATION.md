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

## 七、参考

- STM32H7 Cache 使用注意：AN4839
- Cortex-M7 取指流水线与 Flash ART：AN4891
- Keil AC5 优化开关：ARM Compiler 5 User Guide §4.1
