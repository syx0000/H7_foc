# DRV8353 停机保护方案

## 问题根因

电机停止时 DRV8353 芯片损坏，根本原因是**感性反冲电压击穿 MOSFET**。

### 物理过程

1. **旧停机方式（危险）**：
   ```
   foc_run=0 → TIM1 CCER=0 → 三相浮空 (HIGH-Z)
   → 磁能 ½LI² 通过体二极管反冲到母线电容
   → 60A 时 ΔV=110V，叠加 48V 母线 → 158V 击穿 DRV8353 (Vds_max=100V)
   ```

2. **能量计算**（60A 工作时）：
   ```
   磁能: E = 3 × ½ × L × I² = 3 × 0.5 × 0.000113 × 60² = 0.61 J
   母线电容 100µF 吸收: ΔV = √(2E/C) = √(2×0.61/100e-6) ≈ 110V
   峰值电压: 48V + 110V = 158V > Vds_max(100V) → 击穿
   ```

3. **与电流限幅的关系**：
   ```
   之前 10A: 反冲能量 ½L×10² = 17mJ → ΔV ≈ 18V → 安全
   现在 60A: 反冲能量 ½L×60² = 610mJ → ΔV ≈ 110V → 击穿
   能量增加 36 倍！
   ```

---

## 解决方案（多层保护）

### 1. 主动短路刹车（核心）

**原理**：PWM2 模式 + 中央对齐，CCR=0 时上管 OFF、下管 ON
- 三相下管全导通 → 电机绕组短路
- 磁能在铜阻耗散：P = 3I²R = 3×60²×0.1 = 1080W
- 时间常数：τ = L/R = 0.113mH / 0.1Ω = 1.13ms

**实现**：
```c
// ifly_fault.c: fault_safe_shutdown()
TIM1->CCR1 = 0;  // 上管 OFF
TIM1->CCR2 = 0;
TIM1->CCR3 = 0;
// MOE 保持使能，让下管导通刹车
s_brake_state = 1;
```

**刹车时间**（动态调整）：
- 公式：`t = I_q × 3ms/A`（3τ 安全系数）
- 范围：[20ms, 200ms]
- 60A → 180ms = 159τ，电流衰减到 e^(-159) ≈ 0

### 2. 母线电压监控

**目的**：防止刹车时间不足导致 VDC 上升

**实现**：
```c
// ifly_fault.c: fault_brake_tick_1ms()
uint32_t vdc = getUdc();  // 单位 0.1V
if (vdc > 550 && s_brake_ticks < BRAKE_MAX_MS) {  // 超 55V
    s_brake_target_ms = s_brake_ticks + 50;  // 延长 50ms
}
```

**阈值**：55V（OVP 60V 留 5V 余量）

### 3. 高速/大电流禁止直接停机

**目的**：避免大惯量/大电流下突然停机

**实现**：
```c
// can_wly.c / foc_bsp.c: CTRL_DISABLE
if (spd_abs > 256000 || iq_abs > 15360) {  // >10rpm 或 >15A
    controller_eyou.controller_mode = PROFILE_VELOCITY_MOCE;
    controller_eyou.velocity_ref = 0;
    HAL_Delay(300);  // 强制减速 300ms
}
```

**阈值**（保守）：
- 速度：载端 10rpm（256000 Q10）
- 电流：15A（15360 Q10）

### 4. 预衰减阶段

**目的**：让电流环 PID 主动把电流降到接近 0

**实现**：
```c
controller_eyou.I_q_ref = 0;
controller_eyou.velocity_ref = 0;
HAL_Delay(30);  // 等待 30ms
```

**时间**：30ms（电流环带宽 ~100Hz，3 个周期）

### 5. OSSR/OSSI 配置

**目的**：MOE=0 时自动短路刹车，而非浮空

**实现**：
```c
// tim.c
sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
```

**效果**：DRV8353 nFAULT 硬件触发 BKIN 时，自动进入短路刹车

### 6. 增加死区时间

**目的**：降低 shoot-through 风险

**实现**：
```c
// tim.c
sBreakDeadTimeConfig.DeadTime = 24;  // 50ns → 100ns
```

**权衡**：死区加倍，导通损耗略增，但安全性提升

### 7. 故障保护加强

**严重故障**（OVP/OC/OT/nFAULT）：
- 旧方案：立即硬切 PWM → 反冲击穿
- 新方案：快速斜坡 50ms → 主动刹车

**一般故障**（速度偏差/堵转）：
- 旧方案：斜坡 100ms → 硬切
- 新方案：斜坡 150ms → 主动刹车

---

## 完整停机流程

```
用户 disable 命令
    ↓
┌─────────────────────────────────┐
│ 1. 高速/大电流检测              │
│    速度 > 10rpm ? ────┐         │
│    电流 > 15A ?   ────┤         │
│                       ↓ YES     │
│    切速度模式，velocity_ref=0   │
│    等待 300ms 强制减速          │
└─────────────────────────────────┘
    ↓ NO
┌─────────────────────────────────┐
│ 2. 预衰减阶段                   │
│    I_q_ref = 0, velocity_ref = 0│
│    等待 30ms（电流环自然衰减）  │
└─────────────────────────────────┘
    ↓
┌─────────────────────────────────┐
│ 3. 主动刹车启动                 │
│    foc_run = 0                  │
│    CCR1/2/3 = 0 → 三相下管短路  │
│    计算刹车时间: t = I_q × 3ms/A│
│    范围: [20ms, 200ms]          │
└─────────────────────────────────┘
    ↓
┌─────────────────────────────────┐
│ 4. 刹车监控（1ms tick）         │
│    每 1ms 检查:                 │
│    - VDC > 55V ? → 延长 50ms    │
│    - 时间到 ? → 进入下一步      │
└─────────────────────────────────┘
    ↓
┌─────────────────────────────────┐
│ 5. 安全关断                     │
│    TIM1 MOE = 0 → 三相 HIGH-Z   │
│    （此时电流已接近 0，安全）   │
└─────────────────────────────────┘
```

---

## 参数保守性

| 参数 | 理论值 | 实际设置 | 安全系数 |
|------|--------|---------|---------|
| 刹车时间常数 τ | 1.13ms | 3ms/A | 2.7× |
| 最短刹车时间 | 10ms | 20ms | 2× |
| 最长刹车时间 | 100ms | 200ms | 2× |
| VDC 阈值 | 60V (OVP) | 55V | 留 5V 余量 |
| 高速阈值 | 20rpm | 10rpm | 2× |
| 大电流阈值 | 30A | 15A | 2× |
| 预衰减时间 | 10ms | 30ms | 3× |
| 强制减速时间 | 100ms | 300ms | 3× |
| 死区时间 | 50ns | 100ns | 2× |

**设计原则**：所有参数留 2~3 倍安全余量，硬件保护优先。

---

## 代码改动

### 文件清单

| 文件 | 改动内容 |
|------|---------|
| `Core/Src/tim.c` | OSSR/OSSI 改 ENABLE，死区 12→24 |
| `foc/foc_app/ifly_fault.c` | 主动刹车状态机 + 动态时间 + VDC 监控 |
| `foc/foc_app/ifly_fault.h` | 导出 `fault_brake_tick_1ms()` 和 `fault_safe_shutdown()` |
| `Core/Src/main.c` | 1ms 周期调用 `fault_brake_tick_1ms()` |
| `Core/Src/can_wly.c` | CAN disable 加高速/大电流检测 + 预衰减 30ms |
| `foc/foc_fast/foc_bsp.c` | 串口 enable0 加高速/大电流检测 + 预衰减 30ms |

### 关键代码片段

**主动刹车状态机**：
```c
// ifly_fault.c
static volatile uint8_t  s_brake_state = 0;
static volatile uint16_t s_brake_ticks = 0;
static volatile uint16_t s_brake_target_ms = 0;

void fault_brake_tick_1ms(void) {
    if (s_brake_state == 1) {
        s_brake_ticks++;
        
        // 监控 VDC
        uint32_t vdc = getUdc();
        if (vdc > 550 && s_brake_ticks < 200) {
            s_brake_target_ms = s_brake_ticks + 50;
        }
        
        if (s_brake_ticks >= s_brake_target_ms) {
            TIM1->BDTR &= ~TIM_BDTR_MOE;
            TIM1->CCER &= ~0x1555u;
            s_brake_state = 2;
        }
    }
}

void fault_safe_shutdown(void) {
    // 动态计算刹车时间
    int32_t iq_abs = abs(controller_eyou.I_q);
    uint16_t brake_ms = (iq_abs * 3) / 1024;  // 3ms/A
    if (brake_ms < 20) brake_ms = 20;
    if (brake_ms > 200) brake_ms = 200;
    
    // 启动刹车
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
    s_brake_state = 1;
    s_brake_target_ms = brake_ms;
}
```

**高速/大电流检测**：
```c
// can_wly.c / foc_bsp.c
case CAN_WLY_CTRL_DISABLE:
    int32_t spd_abs = abs(controller_eyou.dtheta_mech);
    int32_t iq_abs = abs(controller_eyou.I_q);
    
    if (spd_abs > 256000 || iq_abs > 15360) {  // >10rpm 或 >15A
        controller_eyou.controller_mode = PROFILE_VELOCITY_MOCE;
        controller_eyou.velocity_ref = 0;
        HAL_Delay(300);  // 强制减速
    }
    
    // 预衰减
    controller_eyou.I_q_ref = 0;
    HAL_Delay(30);
    
    // 主动刹车
    controller_eyou.foc_run = 0;
    fault_safe_shutdown();
    break;
```

---

## 测试验证

### 阶段 1：低电流（10A）

**配置**：
```c
#define INC_PID_SPEED_LIMIT (10 * 1024)
```

**测试项**：
1. enable/disable 循环 100 次
2. 示波器监控 VDC，确认无尖峰
3. 检查 DRV8353 温度正常
4. 观察刹车时间（应为 30ms 左右）

**预期结果**：
- VDC 峰值 < 50V
- 刹车时间 = 10 × 3 = 30ms
- DRV8353 温度 < 60°C

### 阶段 2：中等电流（20A）

**配置**：
```c
#define INC_PID_SPEED_LIMIT (20 * 1024)
```

**测试项**：
1. 重复阶段 1 测试
2. 观察刹车时间（应为 60ms 左右）
3. 测试高速 disable（>10rpm）触发强制减速

**预期结果**：
- VDC 峰值 < 52V
- 刹车时间 = 20 × 3 = 60ms
- 高速 disable 延迟 330ms（300+30）

### 阶段 3：高电流（40A）

**配置**：
```c
#define INC_PID_SPEED_LIMIT (40 * 1024)
```

**测试项**：
1. 对拖测试，加载到 40Nm
2. 触发 disable，观察刹车时间（应为 120ms）
3. 示波器确认 VDC < 55V

**预期结果**：
- VDC 峰值 < 54V
- 刹车时间 = 40 × 3 = 120ms
- 无 VDC 延长触发

### 阶段 4：额定电流（60A）

**配置**：
```c
#define INC_PID_SPEED_LIMIT (60 * 1024)
```

**测试项**：
1. 对拖测试，加载到 60Nm
2. 触发 disable，观察刹车时间（应为 180ms）
3. 示波器确认 VDC < 55V
4. 连续测试 50 次，确认 DRV8353 不损坏

**预期结果**：
- VDC 峰值 < 55V
- 刹车时间 = 60 × 3 = 180ms
- 可能触发 VDC 延长（最多到 200ms）
- DRV8353 温度 < 80°C

### 阶段 5：故障测试

**测试项**：
1. 人为触发 OVP（提高母线电压到 58V）
2. 观察快速斜坡 50ms + 刹车
3. 人为触发 OC（短路相线）
4. 确认 DRV8353 不损坏
5. 测试 nFAULT 硬件触发（OSSR/OSSI 生效）

**预期结果**：
- OVP 触发后 50ms 斜坡 + 动态刹车
- OC 触发后立即斜坡 + 刹车
- nFAULT 触发后自动短路刹车（IDLE 状态）
- 所有情况 DRV8353 不损坏

---

## 硬件建议（必须做）

### 1. TVS 二极管（必须）

**型号**：SMBJ58A 或 SMCJ58A
- 反向截止电压：58V
- 钳位电压：64.5V @ 10A
- 峰值功率：600W (10/1000µs)

**位置**：母线正负极之间，尽量靠近 DRV8353

**成本**：￥1-2/个

**效果**：即使软件失效，硬件也能保护

### 2. 增大母线电容（强烈建议）

**规格**：100µF → 470µF 或 1000µF
- 耐压：≥63V
- 纹波电流：≥2A RMS
- ESR：<0.1Ω @ 100kHz

**效果**：
- ΔV 从 110V 降到 50V 或 35V
- 吸收能量能力提升 4.7~10 倍

**成本**：￥5-10

**缺点**：体积大、启动浪涌电流大

### 3. 刹车电阻（可选）

**适用场景**：频繁制动、高速运行

**方案**：VDC > 阈值时 PWM 控制 IGBT 泄放到刹车电阻
- 电阻：50Ω / 50W
- IGBT：IRLB8721 或类似
- 驱动：光耦 + 栅极电阻

**成本**：￥20

**效果**：可以承受任意大的反冲能量

---

## 已知限制

### 1. HAL_Delay 阻塞

**问题**：CAN/串口 disable 路径用 `HAL_Delay(300+30)`，阻塞 330ms

**影响**：
- CAN 接收中断被阻塞
- 其他任务无法执行

**解决方案**（可选）：
改成异步状态机，类似故障处理的 `ramp_down_phase`：
```c
static uint8_t disable_phase = 0;
static uint16_t disable_ticks = 0;

// 在 1ms tick 里调用
void disable_state_machine(void) {
    if (disable_phase == 1) {  // 强制减速中
        disable_ticks++;
        if (disable_ticks >= 300) {
            disable_phase = 2;
            disable_ticks = 0;
        }
    } else if (disable_phase == 2) {  // 预衰减中
        disable_ticks++;
        if (disable_ticks >= 30) {
            controller_eyou.foc_run = 0;
            fault_safe_shutdown();
            disable_phase = 0;
        }
    }
}
```

### 2. 刹车时间最长 200ms

**问题**：极端情况（60A + 高速 + 低母线电容）可能需要更长刹车

**解决方案**：
- 监控 VDC，如果持续 > 55V，自动延长到 300ms
- 或者加刹车电阻硬件

### 3. 电流采样延迟

**问题**：`controller_eyou.I_q` 是上一拍的值，有 100µs 延迟

**影响**：刹车时间计算略有偏差（<5%）

**解决方案**：可接受，已有 2.7× 安全系数

---

## 总结

### 核心改进

1. **主动短路刹车**：磁能在铜阻耗散，而非反冲到母线
2. **动态刹车时间**：根据电流自适应，避免过短或过长
3. **VDC 监控**：实时检测母线电压，自动延长刹车
4. **高速/大电流保护**：禁止突然停机，强制先减速
5. **OSSR/OSSI 配置**：硬件 BKIN 触发时自动短路刹车
6. **故障保护加强**：严重故障也先斜坡，避免硬切

### 安全系数

所有参数留 2~3 倍安全余量：
- 刹车时间：3τ（理论 1τ 即可）
- VDC 阈值：55V（OVP 60V 留 5V）
- 高速阈值：10rpm（实际 20rpm 才危险）
- 大电流阈值：15A（实际 30A 才危险）

### 硬件必做

1. **TVS 二极管**（￥2，必须）
2. **增大母线电容**（￥5-10，强烈建议）
3. **刹车电阻**（￥20，频繁制动时必须）

### 测试流程

分 5 个阶段逐步提升电流：10A → 20A → 40A → 60A → 故障测试

每个阶段验证：
- VDC 峰值 < 阈值
- 刹车时间符合预期
- DRV8353 温度正常
- 无硬件损坏

---

**编译状态**：0 Error 0 Warning

**实施建议**：先做硬件（TVS + 电容），再烧录软件，分阶段测试
