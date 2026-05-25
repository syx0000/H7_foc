# 100 rpm（输出端）稳定性 + 扭矩输出能力分析

**日期**: 2026-05-25
**工况**: 输出端 100 rpm = 电机端 2500 rpm = 电频率 333 Hz（接近 `DEFAULT_MAX_SPEED` 硬限）

## 1. 物理电压预算

48 V 母线，SVPWM 线性区上限 = 48/√3 ≈ 27.7 V（对应 `INC_PID_CURRENT_LIMIT = 27648 Q10`）。

```
ω_e = 2500 rpm × NPP(8) × 2π / 60 = 2094 rad/s

Iq = 30 A 工况:
  BEMF (ω_e × ψ_f)        = 2094 × 0.00967 Wb       = 20.3 V    ← 主导
  L 项 (ω_e × Lq × Iq)    = 2094 × 0.113 mH × 30 A  = 7.1 V
  R 项 (Rs × Iq)          = 0.076 Ω × 30 A          = 2.3 V
  死区                    ≈ 0.9 V
  ─────────────────────────────────────────────────────
  合计                    ≈ 30.6 V    >  27.7 V (线性区上限)
```

**结论**：30 A @ 100 rpm 已超出 SVPWM 线性区，30 A 以上会被等比缩放（过调制），扭矩输出受限。

实测参数依据（CLAUDE.md "实测参数 已落盘 Flash"）：
- Rs = 0.0764 Ω
- Ld = Lq = 0.113 mH
- ψ_f = 0.00967 Wb
- NPP = 8

## 2. 关键 Bug：BEMF 前馈被 limit_norm 等比砍掉

代码顺序（`foc_current_loop.c:91-163`）：

```
1. PI_QAxis.PidRun()        → V_q (限幅 OutputMax = INC_PID_CURRENT_LIMIT = 27 V Q10)
2. V_q += BEMF_FF            (≈ 20.3 V Q10 @ 100 rpm)
3. limit_norm(V_d, V_q, 27V) → 等比缩放
```

**致命点**：第 1 步 PI 的 `OutputMax` 直接是 27 V，没给 BEMF 留预算。一旦 PI 撞顶，第 2 步加 BEMF 让总和到 47 V，第 3 步 `limit_norm` 等比缩到 27 V。

```
PI 饱和场景:
  PI 出  = 27 V
  +BEMF  = +20 V
  ───────── 47 V
  limit_norm scale = 27 / 47 = 0.57
  V_q 实际        = 27 V        ← 总和不变
  PI 实际贡献      = 0.57 × 27 = 15.4 V
  BEMF 实际贡献    = 0.57 × 20 = 11.4 V   ← 损失 8.6 V
```

**等价于 BEMF 前馈在 PI 不饱和时才有效，PI 撞顶后前馈被砍掉一半**。100 rpm 工况 PI 几乎肯定饱和（电压预算 30.6 > 27 V），所以前馈实际效果打折，扭矩和稳定性都会受影响。

## 3. 次要因素

| 项 | 现状 | 100 rpm 影响 |
|----|------|---------------|
| `DEFAULT_MAX_SPEED = 100 rpm` 输出端 | 撞顶 | 加速时 velocity_ref 被 set_velocity_ref_loop 钳到上限，加速能力丢失 |
| 超速保护 `OVERSPD_LOW = 2600 rpm` 电机端 | `#if 0` 关闭 | 当前不影响；若开启则距工作点仅 4 rpm 输出端裕量 |
| SVPWM 调制度 | ≈ 0.97 (近过调制) | 谐波增大、扭矩纹波加剧 |
| 死区补偿 `DEADTIME_COMP_VOLTAGE = 912` (0.89 V) | 固定值，不随 Iq 缩放 | 30 A 欠补偿、5 A 过补偿 → 过零畸变和谐波 |
| `MIN_ACC_TIME = 1200 ms` | 慢 | 0→100 rpm 用 1.2 s，稳态不影响 |
| `CURRENT_LOOP_MIN_ACC_TIME = 40 ms` | 适中 | 电流斜坡 1 A/ms，稳态不影响 |

## 4. 修复优先级

| P | 项 | 改法 | 预期效果 | 改动量 |
|---|----|------|----------|--------|
| **P0** | PI OutputMax 给 BEMF 留空间 | 拆成 `INC_PID_CURRENT_PI_LIMIT = 12288` (12 V)；`limit_norm` 仍用 27 V | BEMF FF 真正生效，100 rpm 扭矩 +30~50% | 1 常量 |
| P1 | 提高 `DEFAULT_MAX_SPEED` 到 110 rpm | `foc_controller.c:51` 改值 | 给工作点 10% 裕量，加减速不饱和 | 1 行 |
| P1 | 重新标定 ψ_f / Ld / Lq | `bwtest3 → bwtest4` | 前馈系数精度上去 | 跑测试 |
| P2 | 死区补偿电流相关查表 | 三段斜率（小电流线性 / 中段饱和 / 大电流定值） | 谐波降低 | 改函数 |
| P2 | SVPWM 过调制注入零序 | 把线性区扩到 Vdc / √3 × 1.155 ≈ 32 V | 电压上限 +15% | 改 svpwm_calc |

## 5. P0 详细改法

### 5.1 拆常量 (`foc/foc_fast/foc_controller.h:162`)

```c
// 原：
#define INC_PID_CURRENT_LIMIT (27648)  // 27V Q10

// 改：
#define INC_PID_CURRENT_LIMIT      (27648)   // 总电压限幅 27V (limit_norm 用)
#define INC_PID_CURRENT_PI_LIMIT   (12288)   // PI 单独限幅 12V (留 15V 给 BEMF FF + 死区补偿)
```

### 5.2 PID 初始化用新常量 (`foc/foc_fast/foc_data.c:264`)

```c
// 原：
FlashData->Pid_CurrentLimit = INC_PID_CURRENT_LIMIT;

// 改：
FlashData->Pid_CurrentLimit = INC_PID_CURRENT_PI_LIMIT;
```

注意：`foc_controller.c:90` 的静态初始化 `.Pid_CurrentLimit = INC_PID_CURRENT_LIMIT` 也要同步改成 `INC_PID_CURRENT_PI_LIMIT`。

### 5.3 limit_norm 不动

`foc_current_loop.c:65/163` 仍传 `INC_PID_CURRENT_LIMIT`（总电压上限 27 V）。这样：
- PI 自己只能输出 ±12 V
- 加上 BEMF FF (±20 V) + 死区补偿 (±1 V) 后总和 ≤ 33 V
- 最后 limit_norm 钳到 27 V，BEMF FF 仍能实质生效

### 5.4 副作用与风险

| 风险 | 评估 | 应对 |
|------|------|------|
| 低速 (<50 rpm) BEMF 小，PI 限幅 12 V 是否够？ | Rs×60A + 死区 ≈ 5.6 V，够 | 无需特殊处理 |
| 动态超调时 PI 撞 12 V 限幅频繁 | anti-windup 已在 (`func_pid.c`) | 已防护 |
| Flash 已有的 `Pid_CurrentLimit = 27648` 旧值 | 上电 InitFlashData 不会自动覆盖 | 升 `FLASH_STRUCT_VERSION 5→6` 强制重写 |

### 5.5 验证方法

100 rpm 加 30 A 扭矩指令，对比改前改后 `logid 30` 输出：

| 指标 | 改前预期 | 改后预期 |
|------|----------|----------|
| `sat_iq` | 1（撞顶） | 0~1（接近顶但不长撞） |
| `sat_vs` | 1（总电压撞 27V） | 1（仍可能撞，但 BEMF 占比正确） |
| `iq_pid OutPut` | ≈ 27000 | ≈ 12000 |
| `vs_q10` (V_d² + V_q² 开方) | 27000 | 27000 |
| 实际 I_q 跟踪 I_q_ref 差距 | 大（前馈被砍） | 小（前馈真正起作用） |

## 6. 相关代码位置

| 文件 | 行 | 用途 |
|------|----|------|
| `foc/foc_fast/foc_controller.h:162` | INC_PID_CURRENT_LIMIT 定义 | P0 改这里 |
| `foc/foc_fast/foc_data.c:264` | InitDefaultPidData | P0 改这里 |
| `foc/foc_fast/foc_controller.c:90` | flash_data 静态初始化 | P0 改这里 |
| `foc/foc_fast/foc_current_loop.c:91-98` | PI Q-axis 计算 | OutputMax 来源 |
| `foc/foc_fast/foc_current_loop.c:108-120` | BEMF 前馈叠加 | 不动 |
| `foc/foc_fast/foc_current_loop.c:163` | 总 limit_norm | 不动 |
| `foc/foc_fast/foc_data.c:84` | `bemf_omega_e_k = NPP × 2π / (1024 × 60)` | 前馈系数 |
| `foc/foc_fast/foc_controller.c:51` | DEFAULT_MAX_SPEED = 100 rpm | P1 提到 110 |

## 7. 相关参考文档

- `TORQUE_DEFICIT_VERIFY.md` — 33 A 跑飞分析（电压饱和 + anti-windup 缺失）
- `过调制对策backlog.md` — 过调制对策 P1/P2 实施状态
- `速度阶跃损坏风险分析.md` — 速度阶跃硬件机理 + MIN_ACC_TIME 推导
- `CLAUDE.md` "实测参数" + "三环 PID 实测最佳值" — 参数依据
