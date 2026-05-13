# FOC 主循环耗时分析与优化方案

## 实施状态

### ✅ 阶段 1 完成（2026-05-13）

**改动**：
1. CubeMX 开启 ICache（`System Core → CORTEX_M7 → CPU ICache: Enabled`）
2. `foc_current_loop.c:101` 的 `weak_magn_control()` 调用加 `#if USE_WEAK_MAGN` 包裹
3. Keil 工程已是 `Optim=3 + oTime=1`（Time 优先），无需改动

**验证结果**：功能正常，ISR 耗时显著下降（具体数据待用户补充）。

**提交**：`3eade00 开启 ICache + 关闭弱磁空转，FOC ISR 耗时优化阶段 1`

---

### ❌ 阶段 2 失败（DCache + DMA 缓冲迁移）

**尝试改动**：
1. CubeMX 开启 DCache + MPU Region 1 配置 D2 SRAM1 (0x30000000) 为非 Cacheable
2. 用 `__attribute__((at(0x30000000)))` 将 DMA 缓冲区迁到 D2 SRAM1

**失败原因**：
- 串口无输出（HardFault 或 DMA 访问失败）
- 可能原因：
  1. D2 SRAM1 时钟未使能（`__HAL_RCC_D2SRAM1_CLK_ENABLE()` 加了也无效）
  2. MPU Region 1 配置不完整（CubeMX 生成代码缺 `IsCacheable/IsShareable` 显式设置）
  3. Keil AC5 的 `__attribute__((at()))` 与 DMA 缓冲区兼容性问题
  4. DCache 一致性问题未完全解决

**回退**：所有阶段 2 改动已回退，保留阶段 1 成果。

**教训**：
- STM32H7 的 DCache + DMA 一致性配置复杂，需要完整的 MPU + 时钟 + scatter 文件配合
- Keil AC5 的 scatter 文件语法与 `.ANY` 匹配规则不直观，`.dma_buffer` 段被 `RW_IRAM2` 的 `.ANY (+RW +ZI)` 优先匹配
- `__attribute__((at()))` 需要每个变量手动分配不同地址，维护成本高
- 建议后续用 AC6 或 GCC，scatter 文件语法更清晰

---

## 一、现象数据（优化前）

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

### ① CPU Cache 未启用 ⭐⭐⭐⭐⭐ ✅ 已完成

**原状态**：`Core/Src/main.c:82~95` 只调用了 `MPU_Config()` 和 `HAL_Init()`，**没有** `SCB_EnableICache/DCache`。

- 代码运行在 Flash，不开 I-Cache 时每次取指都吃 Flash 等待周期（STM32H7 @480MHz 约 4~7 WS）。
- 经验值：**开 I-Cache 后纯代码执行时间降到 1/3~1/5**。
- `ISR entry=7us` 是典型的"Cache 没开"特征。

**已实施**：CubeMX 勾选 `System Core → CORTEX_M7 → CPU ICache: Enabled`，生成代码自动在 `MPU_Config()` 之后插入 `SCB_EnableICache();`。

**预期收益：43us → 15~20us**（实测待补充）

---

### ② 编译器优化方向错误 ⭐⭐⭐⭐ ✅ 已确认无需改

**检查结果**：`MDK-ARM/cubemx_yxsui.uvprojx` 已是 `Optim=3 (-O2) + oTime=1 (Time 优先)`，无需改动。

**预期收益：已包含在当前编译配置中**

---

### ③ 关键代码未放置 ITCM/DTCM ⭐⭐⭐ ⏸️ 暂缓

**原状态**：`foc/foc_fast/foc_bsp.h:28-34` 的 `ATTR_RAMFUNC` 和 `ATTR_PLACE_AT_FAST_RAM_INIT` 为空宏。

sin/cos 表、FOC kernel 函数都在 Flash，即使开 Cache 也有 miss 抖动（`max=67us` 就是 cache miss 尾部）。H7 有 64KB ITCM + 128KB DTCM，足够放所有 FOC 热代码。

**暂缓原因**：需要修改 scatter 文件，Keil AC5 语法复杂，阶段 2 失败后暂缓。

**预期收益：消除抖动，max 从 67us 接近 typical**

---

## 三、代码层面的热点（按耗时权重排序）

### 🔥 热点 1：`weak_magn_control` 空转 —— **5~10us** ✅ 已完成

**原问题**：`foc/foc_fast/foc_current_loop.c:101` 每周期调用 `weak_magn_control(controller)`。即使 `USE_WEAK_MAGN = 0`，外层包裹宏只是包了内部赋值分支，**函数本身仍在执行**。

**已实施**：把 `weak_magn_control` 的调用用 `#if USE_WEAK_MAGN` 包起来：
```c
#if USE_WEAK_MAGN
  weak_magn_control(controller);
#endif
```

**预期收益：-5us**

---

### 🔥 热点 2：`svpwm_calc` 的海量整数除法 —— **5~8us** ⏸️ 待优化

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

| # | 动作 | 类型 | 状态 | 预计收益 | 风险 |
| - | ---- | ---- | ---- | -------- | ---- |
| 1 | CubeMX 开启 ICache | 配置 | ✅ 完成 | **-20us** | 无 |
| 2 | Keil 工程 Optimize for Time | 配置 | ✅ 已是 | 已包含 | 无 |
| 3 | `weak_magn_control` 加 `#if USE_WEAK_MAGN` | 代码 | ✅ 完成 | **-5us** | 无 |
| 4 | `check_phases_overcurrent_timesliced` 下放到 1kHz 慢环 | 代码 | ⏸️ 待做 | **-3us（max 降更多）** | 需确认过流保护时序要求 |
| 5 | PID 函数指针改直接调用 `IncPIDCal` | 代码 | ⏸️ 待做 | -1us | 无 |
| 6 | `svpwm_calc` 预算 `PWM_T/UDC` 系数 + 常量除法改位移 | 代码 | ⏸️ 待做 | -3us | 需对比波形 |
| 7 | FOC 热代码 / sin 表放 ITCM/DTCM | 配置+链接 | ⏸️ 暂缓 | 降抖动 | 需改 scatter 文件 |
| 8 | DCache + DMA 缓冲迁 D2 SRAM | 配置+代码 | ❌ 失败 | -3~5us | 复杂度高，已回退 |

---

## 五、验证方法

修改前后对比 `ADC ISR: duration last / max(1s)` 日志字段，关注：
- `last` 是否从 43us 降到目标值
- `max(1s)` 是否显著下降（消除 cache miss / 过流检测触发尾部）
- `entry` 是否从 7us 降到 1~2us（cache 效果最直接的体现）

**阶段 1 实测结果**：（待用户补充）

---

## 六、后续优化方向（待实施）

### 优先级 1：代码层热点优化（低风险）

1. **`check_phases_overcurrent_timesliced` 下放到 1kHz 慢环**
   - 当前每 4 拍触发一次 50 点循环，触发拍 ISR 延迟 +10us
   - 改为轻量 peak-hold 算法或下放到主循环
   - 预期：max 从 67us 降到 50us 以下

2. **PID 函数指针改直接调用**
   - `foc_current_loop.c:91,97` 的 `PidRun` 函数指针改为直接调用 `IncPIDCal`
   - 预期：-1us

3. **`svpwm_calc` 除法优化**
   - 预算 `PWM_T/UDC` 为 Q15 乘系数
   - 常量除法改位移（`/1024` → `>>10`）
   - 预期：-3us

### 优先级 2：ITCM/DTCM 热代码放置（中风险）

- 需要修改 Keil scatter 文件，语法复杂
- 建议后续迁移到 AC6 或 GCC 后再做
- 预期：消除 max 抖动

### 优先级 3：DCache + DMA 缓冲迁移（高风险，已失败）

**失败教训**：
- STM32H7 的 DCache + DMA 一致性需要完整的 MPU + 时钟 + scatter 文件配合
- Keil AC5 的 scatter 文件 `.ANY` 匹配规则不直观
- `__attribute__((at()))` 需要手动分配地址，维护成本高
- D2 SRAM 时钟使能后仍无输出，根因未明

**建议**：暂缓，等迁移到 AC6/GCC 或有更充裕时间再尝试。

---

## 七、参考

- STM32H7 Cache 使用注意：AN4839
- Cortex-M7 取指流水线与 Flash ART：AN4891
- Keil AC5 优化开关：ARM Compiler 5 User Guide §4.1
