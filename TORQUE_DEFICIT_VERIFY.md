# 扭矩不达问题验证计划

## 1. 目标

定位"实际扭矩跑不到额定（65Nm 输出端 / ~22A 电机端）"的根因，并验证修复。

## 2. 嫌疑清单（按概率）

| # | 嫌疑 | 文件:行 | 当前值 | 期望 |
|---|------|--------|-------|------|
| S1 | BEMF 前馈被关闭，高速 Vq 全靠 PI 撑 | `foc_controller.h:46` | `USE_BEMF_FF=0` | `1` |
| S2 | 速度环输出限幅 60A 远超电机额定 22A，导致 Iq_ref 长期 ≫ 实际可达 | `foc_controller.h:156` | `INC_PID_SPEED_LIMIT=60×1024` | `DEFAULT_MAX_CURRENT` |
| S3 | 电流环斜坡换算按 4kHz 推但实际跑 10kHz，斜率偏 2.5× | `foc_current_loop.c:389` | `(* 4)` | `(* 10)` |
| S4 | 死区补偿被关 | `foc_controller.h:60` | `USE_DEADTIME_COMPENSATION=0` | `1` |
| S5 | 超速降压代码块被 `#if 0` 注释 | `foc_current_loop.c:129` | 关 | 调试结束后开 |
| S6 | `I_a_Filter/I_b_Filter/I_c_Filter` 三相过流保护可能未喂数据，事实上失效 | `foc_current_loop.c:484` | 待查 | 修或注释掉 |

## 3. 准备工作

### 3.1 上电检查（每次抓数据前必做）

```
logid165          # 查故障，应是 0
logid163          # 有故障先清
logid162          # Dump RAM vs Flash，确认 Rs/Ld/Lq/ψ_f 都已落盘
```

期望开机 log（参考 CLAUDE.md "正常开机 log 关键行"）：
```
ADC calibration done: Off_a=~32768 Off_b=~32768
Motor params loaded from Flash: Rs=0.0794 Ohm  Ld=0.1133 mH  Lq=0.1163 mH
FOC initialization done, NPP=8, foc_run=2
```

### 3.2 串口抓包

```bash
powershell -ExecutionPolicy Bypass -File tools/capture_com.ps1 \
    -PortName COM4 -Baud 921600 -Seconds 30 -OutFile case_<N>.log
```

每个用例抓 30s，留 10s 稳态 + 10s 阶跃 + 10s 后续。

## 4. 测试用例

### 用例 A — 基线复现（确认问题存在）

```
logfreq50         # 20Hz 抓 logid 200 (内部已节流到 50ms)
logid200
RuncmdXM4Y22000   # 进 PROFILE_TORQUE_MODE，目标 ~21.5A (22000/1024)
```

**预期**：实际 Iq 远低于 22000，扭矩传感器读数也低于额定。如果 Iq 能跟到位但负载扭矩仍不够，方向跑偏到机械/磁链估算。

### 用例 B — 低速纯电流环（隔离 BEMF）

电机堵转 / 极低速（<5 rpm 电机端），ω_e ≈ 0，Vq_bemf 可忽略：

```
logid200
RuncmdXM4Y10240   # 10A
RuncmdXM4Y22000   # 22A
```

**判据**：低速下若 Iq 能跟上 → BEMF FF 是高速失配元凶（S1）。低速也跟不上 → 直接看 [L200/3] sat 标志。

### 用例 C — 高速负载（暴露 S1）

```
logid200
RuncmdXM3Y50000   # 进速度模式，目标 ~48 rpm load端
```

加机械负载到拉到额定扭矩附近，看 mod% 和 Vs 撞限。

### 用例 D — 修一个测一个（增量验证）

按 S1 → S2 → S3 → S4 顺序逐个改，每改一项重跑用例 A + C，对比 Iq 上限和 mod% 变化。

## 5. 数据分析判据

按 [L200/n] 行号查：

### [L200/1] 模式 / 故障 / 速度链
- `mode` 不是预期模式（4=PROFILE_TORQUE / 8=CSP / 11=MIT）→ 命令没生效，重发 Runcmd
- `err != 0` → 故障被触发，先清错再继续
- `smech` 跟 `sref` 反号 → 相序错，重跑 `Cali`
- `Imax` 不等于预期（Q10）→ MaxCurrent 被外层夹了

### [L200/2] 速度环 PID
- `sat=±1` 且 `Iq_ref` 已到 `Imax` → 速度环饱和，问题往电流环找
- `sat=±1` 且 `Iq_ref` 远小于 `Imax` → `INC_PID_SPEED_LIMIT` 把输出夹了（**S2 直接命中**）
- `err` 大但 `out` 小 → Kp 太小

### [L200/3] 电流环 + 电压裕量 + BEMF
- `sat:Iq=±1` + `Iqerr` 大 → 电流环 PI 顶限但跟不上，看 Vs 是不是同时撞限
- `sat:Vs=1` + `mod%≈100` → 电压不够，**S1 命中**：开 BEMF FF 后 mod% 应下降 5~15%
- `mod%>100%` → 过调制，SVPWM 已经在线性区外，电流跟随会失真
- `Vq_bemf` 占 Udc 比例大（>30%）但 `BEMF_ff=0` → 必然要开 FF
- `psi=0` 或 `Lq=0` → Flash 参数没读上来，重跑 `bwtest3`/`bwtest4`

### [L200/4] 三相反馈 + αβ + 原始 ADC
- `Ia+Ib+Ic` 不接近 0（>1A） → 采样链路或 offset 校准有问题
- `raw a/b` 离 `offA/offB` 相差很小（<100）但电机在跑 → 采样窗口对不上 PWM 中点
- `theta_e` 跟实际机械位置不同步 → `Cali` 没做 / 编码器丢包

### [L200/5] PWM CCR + 双环斜坡
- 单相 CCR 贴近 0 或 PWM_T(24000) → SVPWM 撞调制极限（mod% 之外的二次确认）
- `SpdRamp now != velocity_ref` 长期不收敛 → `MIN_ACC_TIME=1200ms` 在限速度参考
- `CurRamp now != I_q_ref` → `CURRENT_LOOP_MIN_ACC_TIME=10ms` 配合 `INC_PID_SPEED_LIMIT=60` 让斜率偏大（**S3 命中**）

### [L200/6] 外层限幅 + 输出端 + 位置环
- `MaxSpd` Q10 单位（load端 rpm × 1024）远小于预期 → 上层 set_velocity_ref 把参考夹了
- `OverI` 比 `Imax` 还小 → fault 层会先于 PI 触发过流
- 位置模式下 `PosPID sat=±1` → 位置环输出已撞 `INC_PID_POSITION_LIMIT`，速度参考被钳

## 6. 根因决策树

```
扭矩不达
├── [L200/1] err != 0
│   └── 先清错，重跑（不是本计划范围）
├── [L200/2] SpdPID sat=1, Iq_ref << Imax
│   └── S2: INC_PID_SPEED_LIMIT 太小 → 改 DEFAULT_MAX_CURRENT
├── [L200/3] sat:Vs=1, mod%≈100
│   ├── BEMF_ff=0
│   │   └── S1: 开 USE_BEMF_FF=1
│   └── BEMF_ff=1, 仍撞限
│       └── 母线电压不够 / 弱磁需求 → 开 USE_WEAK_MAGN
├── [L200/3] sat:Iq=1, Iqerr 大, Vs 还有裕量
│   └── 电流环增益不够 → 重跑 bwtest6
├── [L200/4] Ia+Ib+Ic 不为 0
│   └── ADC offset 异常，重跑 ADC_CalibrateOffsets
└── [L200/4] theta_e 跟机械角度不同步
    └── 重跑 Cali（电角度偏置辨识）
```

## 7. 修复 → 回归

每次修一项后跑一次：

```
# 1. 编译烧录
"C:/Keil_v5/UV4/UV4.exe" -f "MDK-ARM/cubemx_yxsui.uvprojx"

# 2. 抓 30s 基线
powershell -ExecutionPolicy Bypass -File tools/capture_com.ps1 \
    -PortName COM4 -Baud 921600 -Seconds 30 -OutFile fix_<S>_baseline.log

# 3. 板上重发命令
logid165
logid163
logid200
RuncmdXM4Y22000

# 4. 加载阶跃，抓另一段 30s
```

**通过判据**：
- Iq 实测峰值 ≥ 21A（Q10 ≥ 21504）
- mod% 稳态 ≤ 90%
- 扭矩传感器读数 ≥ 60Nm 输出端
- 无新故障

## 8. 修复完成后清理

- S5 的 `#if 0` 改回 `#if 1`，恢复超速降压保护
- 如果 S6 的过流保护链路确实失效，要么修 `I_a_Filter` 赋值，要么把 `check_phases_overcurrent_timesliced` 整段 `#if 0` 掉以省 ISR 时间
- CLAUDE.md "电流环前馈与保护参数" 段更新最终配置
- Flash 落盘最终参数：`logid160`

## 9. 抓数据快查表

| 命令 | 作用 |
|------|------|
| `logid200` + `logfreq50` | 启动诊断快照（20Hz） |
| `logid200` + `logfreq30` | 抓动态阶跃（33Hz, 极限） |
| `logid0` | 关闭打印 |
| `logid165` | 查故障 |
| `logid163` | 清故障 |
| `logid162` | RAM vs Flash 参数对比 |
| `RuncmdXM4Y<iq_q10>` | 进扭矩模式给 Iq 指令 |
| `RuncmdXM3Y<rpm×1024>` | 进速度模式给速度指令 |
| `enable0` / `enable1` | PWM 失能 / 使能 |

---

## 10. 实测数据分析 (2026-05-20)

### 10.1 CSV 数据: 高速加载 33A 跑飞 (vofa2+.csv)

**工况**: 高速运行，电流持续爬升（速度环加速中）

**电流增长趋势** (每100点采样峰值, 10kHz采样率):

| 行号 | 峰值电流 | 状态 |
|------|---------|------|
| 43800 | 20A | 正常正弦，三相对称 |
| 44200 | 21A | 开始增长 |
| 44500 | 24A | 持续增长 |
| 44800 | 27A | 加速增长 |
| **44999** | **33A** | **临界点** |
| 45025 | 35A | 波形畸变，三相不对称 |
| 45046 | 54A | 完全失控 |
| 45063 | 134A (ADC饱和) | 跑飞 |
| 47484 | 0 | PWM关断 |
| 47485+ | 固定偏置 | 停机 |

**跑飞逐拍过程** (行 45025~45063, 仅 38 点 = 3.8ms):

```
45025: 20130, 15303, -35433   ← I2突然偏大 (正弦应~33A, 实际35A)
45026: 16005, 20422, -36427   ← I2继续偏大
45029: -2743, 31329, -28586   ← I1跳到31A, 波形已不对称
45030: -8262, 35924, -27662   ← I1=35A, 完全畸变
45037: -35833, 19692, 16141   ← 幅值38A
45042: -35095, -3947, 39042   ← I2=39A (三相和仍=0, 采样正常)
45046: -25781, -28780, 54561  ← 54A, 完全失控
45058: 73115, -36106, -37009  ← 73A
45063: 134904, 33099, -168003 ← ADC饱和 (134904=满量程)
```

**关键特征**: 三相电流和始终≈0 (采样无故障), 但波形从对称正弦突变为不对称 → 电压饱和后电流环失控。

### 10.2 串口 log: 低速堵转 Vd 异常 (48v 10rpm 关电压前馈.txt)

**工况**: 48V 母线, ~25rpm 电机端 (堵转), BEMF前馈关闭, 电流从1A爬到19A

**Vd/Vq 随电流变化**:

| Iq_ref | Vq (Q10) | Vd (Q10) | |Vd|/Vq | 等效角度偏差 |
|--------|----------|----------|---------|------------|
| 1A | 2200 | +80 | 4% | ~2° (正常) |
| 10A | 2900 | -700 | 24% | ~14° |
| 14A | 2700 | -1600 | 59% | ~30° |
| 19A | 1850 | **-2450** | **132%** | **53°** |

**关键观察**:
- Iq 跟踪正常: Iq≈Iqref, Iqerr<200 (电流环本身没问题)
- Vd 随电流**线性增大**, 从+80到-2450
- 调制度始终很低: mod=7~11% (Vs=2000~3200, Vlim=28467, 远未饱和)
- 空载 100rpm (电机端2500rpm) 运行正常 → **静态电角度偏置正确**

### 10.3 排除电角度偏置

空载高速正常运行证明:
1. 电角度辨识值 (elec_offset) 正确
2. 编码器通信无丢包
3. Park/Clarke 变换方向正确

Vd 随电流增大不是角度错, 是**电流相关的系统性畸变**。

### 10.4 采样电阻对比实验 (关键证据)

| 采样电阻 | 固件报告跑飞电流 | 实际物理电流 | 跑飞时扭矩 |
|----------|----------------|------------|-----------|
| 25mΩ (原) | 33A | 33A | T |
| 12.5mΩ (半) | **33A** | **66A** | **2T** |

固件电流标度未随采样电阻更新，12.5mΩ时固件报告值=实际值/2。

**关键结论**:
1. **跑飞点由固件 Q10 数值决定，非物理电流/电压极限** — 否则 12.5mΩ 时实际 66A 需要更多电压，应在更低的固件报告值就饱和
2. **排除磁饱和** — 如果是铁芯饱和导致失控，应在相同物理电流触发，而非相同 Q10 值
3. **排除热效应** — 66A 的铜损是 33A 的 4 倍，如果是热导致应更早触发
4. **指向固件数字域阈值** — 某个与 Q10 电流值相关的计算在 33A (33792 Q10) 附近触发失稳

**可能的固件域触发机制**:
- 电流环 PI 积分器累积到 OutputMax (28467) 时，增量式 PID 无 anti-windup → 正反馈
- 速度环在相同的加速时间内把 I_q_ref 推到相同的 Q10 值（因为速度环不感知采样电阻变化）
- BEMF 前馈关闭时，PI 输出 = Rs_firmware×I_q + BEMF，其中 Rs_firmware 是固定值，所以 PI 输出在相同 Q10 电流时达到相同值

**补充解释为何两种电阻跑飞点相同**:
速度环输出 I_q_ref 的斜坡速率由 `CurrentLoopSmoothRun` 决定（固定 Q10/tick），与采样电阻无关。两种情况下 I_q_ref 以相同速率爬升到 33A Q10，此时电流环 PI 输出（电压指令）也达到相同值，触发 limit_norm 截断 → 积分饱和 → 跑飞。实际物理电流不同但固件"看到"的数字世界完全一致。

## 11. 根因定位

### 11.1 死区 + 器件压降畸变 (主因)

死区效应在 dq 坐标系下的基波分量:

```
Vd_deadtime = -(6/π) × Vdt

Vdt = Vdc×Td/Ts + Rds_on×I + V_diode
    = 48×100ns/50μs + 5mΩ×I + 0.7V
    = 0.096 + 0.005×I + 0.7
```

| 电流 | Vdt | 理论 Vd_dt | 实测 Vd | 差值(未补偿残余) |
|------|-----|-----------|---------|----------------|
| 1A | 0.80V | -1.53V (-1567 Q10) | +80 | 补偿有效 |
| 19A | 0.89V | -1.70V (-1741 Q10) | -2450 | **欠补偿 709 Q10** |

实测 Vd 比理论更大, 说明:
- 实际器件压降 > 估算 (Rds_on 热漂移, 体二极管反向恢复)
- 或死区补偿参数偏小

### 11.2 死区畸变如何导致 33A 跑飞

```
电流增大
  → 死区畸变 Vd 增大 (与电流线性)
  → 总电压 Vs = √(Vq² + Vd²) 增大
  → Vs 撞到 limit_norm 上限 (INC_PID_CURRENT_LIMIT=28467)
  → Vq 被压缩 (等比缩放)
  → 实际 Iq 跟不上指令
  → 电流环积分器继续累积 (无 anti-windup)
  → 正反馈 → 2ms 内跑飞
```

### 11.3 电压预算分析 (33A 临界点)

48V 母线, 线性区电压上限 = 48/√3 = 27.7V ≈ 28467 Q10 (= INC_PID_CURRENT_LIMIT)

| 分量 | 估算值 | 说明 |
|------|--------|------|
| Rs×Iq | 0.076×33 = 2.5V | 电阻压降 |
| ω_e×ψ_f (BEMF) | ~16V @2000rpm | 反电动势 (关前馈时PI承担) |
| 死区 Vd | ~3V @33A | 死区畸变 |
| **总计** | **~21.5V** | 占 27.7V 的 78% |

看似有裕量, 但:
- PI 动态超调 + 死区 Vd 占用 → 实际 Vs 瞬间撞限
- 一旦撞限, 增量式 PID 无 anti-windup → 积分器爆炸

### 11.4 代码层面缺陷

| 缺陷 | 文件:行 | 影响 |
|------|--------|------|
| 增量式PID无anti-windup | `func_pid.c:39-48` | OutPut 触顶后积分仍累积 |
| 速度环限幅60A远超实际能力 | `foc_controller.h:156` | 允许请求不可能达到的电流 |
| limit_norm 等比缩放无反馈 | `foc_kernel.c:239-246` | 电流环不知道自己饱和了 |
| 死区补偿参数可能偏小 | `foc_current_loop.c:170` | 高电流欠补偿 |
| UDC 硬编码常量 | `foc_bsp.h:76` | SVPWM 不跟踪实际母线电压 |

## 12. 修复优先级

| 优先级 | 修复项 | 预期效果 |
|--------|--------|---------|
| P0 | 电流环 anti-windup: Vs 饱和时冻结积分 | 防止跑飞 |
| P0 | 速度环限幅降到 30A (30720 Q10) | 限制电流指令在可控范围 |
| P1 | 校准死区补偿参数 (增大 Vdt 估算) | 减小 Vd 畸变, 恢复扭矩 |
| P1 | 开启 BEMF 前馈 (USE_BEMF_FF=1) | 释放 PI 带宽给死区补偿 |
| P2 | UDC 改为实时采样值 | SVPWM 精度提升 |
| P2 | 过调制检测: Vs>0.95×Vlim 时报警 | 提前预警 |

## 13. 已实施修改 (2026-05-20)

### 13.1 P0 — 防跑飞

| 文件 | 修改 | 说明 |
|------|------|------|
| `func_pid.h:42` | IncPID 增加 `uint8_t saturated` 字段 | anti-windup 标志位 |
| `func_pid.c:30-40` | 饱和时冻结积分项 (只保留P+D) | 防止积分器爆炸 |
| `foc_current_loop.c:158-165` | limit_norm 后检测截断，回写 saturated | 电压饱和→冻结积分 |
| `foc_controller.h:156` | `INC_PID_SPEED_LIMIT` 60A→**35A** | 限制电流指令上限 |

### 13.2 P1 — 恢复扭矩

| 文件 | 修改 | 说明 |
|------|------|------|
| `foc_controller.h:46` | `USE_BEMF_FF` 0→**1** | 开启反电动势前馈 |
| `foc_controller.h:60` | `USE_DEADTIME_COMPENSATION` 0→**1** | 开启死区补偿 |
| `foc_controller.h:63` | `DEADTIME_COMP_VOLTAGE` 49→**912** | 补偿电压校准 (0.89V) |
| `foc_current_loop.c:124` | 改用 `deadtime_compensation_3phase` | 三相版本精度更高 |

### 13.3 保护恢复与校准

| 文件 | 修改 | 说明 |
|------|------|------|
| `foc_current_loop.c:131` | 超速保护 `#if 0`→`#if 1` | 恢复 2600/2700rpm 阈值 |
| `foc_current_loop.c:340` | 补上 `I_a/b/c_Filter` 一阶低通赋值 | 三相过流保护数据源生效 |
| `ifly_fault_api.c:32` | `BlockTorque` 13A→**38A** | 堵转保护不误触发 |
| `ifly_fault_api.c:35` | `OverCurrent` 64A→**40A** | 母线过流阈值对齐 |
| `ifly_fault_api.c:38` | `UVWCurrentLimit` 6860→**46080** (45A Q10) | 三相过流阈值单位修正 |

### 13.4 保护层级设计

```
正常运行
  │
  ├─ 35A ── 速度环限幅 (INC_PID_SPEED_LIMIT)
  │         正常运行电流上限，anti-windup 冻结积分
  │
  ├─ 38A ── 堵转保护 (BlockTorque)
  │         I_q > 38A 且 speed < 101rpm 电机端，连续 30ms → 停机
  │
  ├─ 40A ── 母线过流 (OverCurrent)
  │         |I_q| > 40A，连续 10ms → 停机
  │
  ├─ 45A ── 三相过流 RMS (UVWCurrentLimit)
  │         单相 RMS > 45A，连续 20 窗口 (~12s) → 停机
  │
  └─ 硬件 ─ DRV8353 nFAULT (TIM1 BKIN)
            过流/过温/VDS 硬件比较器 → 立即关 MOE
```
