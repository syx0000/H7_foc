# 弱磁 + 缺相保护 + 电流环重构 验证计划

**日期**: 2026-05-26
**变更范围**: 弱磁控制实现、电流环时序重构、BEMF FF 重启用、缺相保护实现
**前置工作**: commit 16be832 (PI 限幅 12V→15V 修复位置环震荡)

---

## 1. 改动总览

### 1.1 文件清单

| 文件 | 行数变化 | 主要内容 |
|------|----------|---------|
| `foc/foc_fast/foc_controller.h` | +18/-3 | USE_BEMF_FF 1, USE_WEAK_MAGN 1, 弱磁参数宏 |
| `foc/foc_fast/foc_current_loop.c` | +175/-44 | I_q_ref 流水线重构, weak_magn_control 重写, BEMF FF 用 compensation_weak |
| `foc/foc_fast/foc_bsp.c` | +13 | logid 200 加 [L200/4b] 弱磁状态行 |
| `foc/foc_app/ifly_fault.c` | +210/-4 | phaseLossProFunc 实现, 严重故障 mask 加缺相位 |
| `foc/foc_app/ifly_fault.h` | +3 | phaseLossProFunc / motorTempProFunc / MotorTemperatureInquiry 声明 |
| `foc/foc_app/ifly_fault_api.c` | +8 | Threshold 默认值加缺相参数 + 电机温度 |
| `foc/foc_app/ifly_fault_api.h` | +9 | Portection_Value 加 4 个缺相阈值字段 + TemMortorWarn |
| `Core/Src/main.c` | +2 | 1ms 调度加 motorTempProFunc + phaseLossProFunc |
| `Core/Src/can_wly.c` | ±1 | 修电机温度告警比对项 (TemBoradWarn → TemMortorWarn) |

### 1.2 编译尺寸

```
基线 (commit 16be832):  Code=87372 (修复 PI 限幅前)
重构后:                  Code=95364
增量:                   +7992 字节
```

---

## 2. 三大功能模块改动详解

### 2.1 电流环时序重构 (foc_current_close_loop)

#### 旧顺序问题
1. `weak_magn_control` 在 PI **之后**调用,`compensation_weak` 滞后 1 拍
2. `CurrentLoopSmoothRun` 在弱磁圆限幅**之前**, 导致圆限幅引起的突变给 PI 阶跃
3. sweep/CAN 注入信号叠加位置不当, 受斜坡平滑导致 bwtest 高频衰减
4. BEMF FF Vq_ff 用 `I_d_ref` (恒为 0), 漏算弱磁工作时 ωe·Ld·Id 项

#### 新流水线 (5 步)
```
速度环输出 I_q_ref
  ↓
[step0] weak_magn_control()              用上拍 V_dq 算本拍 id_weak
  ↓
[step1] iq_basic = I_q_ref               基础指令 (不含扰动)
  ↓
[step2] 弱磁圆限幅 iq_basic              命中→IncPID_Speed.saturated=1
  ↓
[step3] CurrentLoopSmoothRun(iq_basic)   斜坡平滑 (不影响 sweep)
  ↓
[step4] + sweep + can_test               扰动叠加在斜坡之后
  ↓
[step5] 总电流圆限幅 (保护性)            不触发 anti-windup
  ↓
PI-Iq AimValue = I_q_ref_filterd
```

### 2.2 弱磁控制 (方案 A: 电压反馈式)

#### 算法
```
1. Vs = √(V_d² + V_q²)         实际电压模长
2. Us_filt = LPF(Vs, α=1/16)   一阶低通去抖
3. vs_excess = Us_filt - 95% × g_vs_limit   触发判据
4. 增量 PI: delta = -(Kp×Δexcess + Ki×excess) / Div
5. id_weak ∈ [-3A, 0]          退磁保护
6. 低速 (|ωe|<100 rad/s) 直接清零 id_weak
7. 退出弱磁: LEAK_OUT_STEP 缓慢回到 0
```

#### 与 BEMF FF 协作
- BEMF FF 占 ~80% 电压圆 (公式前馈, 减小 PI 负担)
- 弱磁在剩余 15% 余量内工作 (撞顶才介入)
- 共用同一份 `omega_e_filt` (来源 `velocity_ref_filterd / 25`, 指令侧)

### 2.3 BEMF 前馈重新启用 (USE_BEMF_FF = 1)

#### 关键改动
1. **ωe 来源换成指令侧**: `velocity_ref_filterd / 25` (载端→电机端)
   - 解决反向切换时 `dtheta_mech` 滞后 1.6~2.3ms 导致的方向错位
2. **低速钳制**: 电机端 |speed| < 30 rpm 时 omega_e_filt = 0
3. **解耦项用指令侧电流**: `I_q_ref_filterd / I_d_ref` 避免反馈噪声经 ωe·L 放大
4. **Vq_ff 用 compensation_weak** (弱磁工作时)
5. **软限幅**: |Vff| ≤ 0.85 × g_vs_limit
6. **动态 PI OutputMax**: g_vs_limit - |Vff_pred| - 死区

### 2.4 缺相保护

#### 4 个并行判据
| 判据 | 条件 | 标志位 |
|------|------|--------|
| J1 KCL | \|Ia+Ib+Ic\| > 1A 持续 100ms | PhaseUVolErr (兜底) |
| J2 A 相低 | 运行 (\|Iq\|>2A) 时 \|Ia\|<0.3A 持续 100ms | PhaseUVolErr |
| J3 B 相低 | 同 J2, B 相 | PhaseVVolErr |
| J4 C 相低 | 同 J2, C 相 | PhaseWVolErr |

#### 集成点
- 1ms 调度 (`main.c:246`)
- 加入严重故障 mask (立即关 PWM, 不走斜坡减速)
- 复用现有打印 [14]/[15]/[16]

---

## 3. 验证准备

### 3.1 工具
- 串口工具 (波特率 921600, COM4, 用 `tools/capture_com.ps1`)
- 烧录: `UV4 -f MDK-ARM/cubemx_yxsui.uvprojx`
- 万用表 (测三相导通)
- vofa+ (实时波形分析, 可选)

### 3.2 编译烧录
```bash
"C:/Keil_v5/UV4/UV4.exe" -r "MDK-ARM/cubemx_yxsui.uvprojx" -j0
"C:/Keil_v5/UV4/UV4.exe" -f "MDK-ARM/cubemx_yxsui.uvprojx"
```

### 3.3 上电健康检查
预期开机 log:
```
LT H7 foc start
ADC calibration done: Off_a=326xx Off_b=329xx
Flash struct version mismatch (got 5, expect 6), force reinit  ← v5→v6 触发
Flash: PhaseOrder=0, mech_offest_out=0, elec_offset=xxxxx
FlashData: CurPID=55/3/0 SpdPID=1500/12/0 PosPID=800/5/0 FF=300
Motor params loaded from Flash: Rs=0.0794 Ohm  Ld=0.1133 mH  Lq=0.1163 mH
psi_f=0.0092 Wb                                                 ← 必有, 否则 BEMF FF 不工作
FOC initialization done, NPP=8, foc_run=2
```

**如缺 `psi_f=...`**: 跑 `bwtest4` 重做磁链辨识, 否则 BEMF FF 和弱磁都进不了启用条件 (`flux_psi > 0.0f`)。

---

## 4. 分阶段验证

### 阶段 0: 基线回归 (排除重构破坏现有功能)

#### T0.1 静止稳定性
```
开机后什么都不发, 等 30 秒
```
**判据**:
- 无故障打印 (无 `FAULT! ServoErrFlag=`)
- `logid 200` 看 [L200/4b]: id_weak=0, vs_excess<0
- 三相电流 [L200/4]: |Ia|, |Ib|, |Ic| < 200 (Q10, ~0.2A 噪声)

#### T0.2 低速速度模式
```
logfreq 500
logid 200
Runcmd2M3tar30        # 30rpm 输出端
等 5 秒稳定
```
**判据**:
- [L200/3] V_d 波动范围 (角度对齐基础检查)
- [L200/4b] WMAG=1 但 id_weak=0 (低速不弱磁)
- [L200/2] 速度环 sat=0 (无积分饱和)

**指标记录**:
| 量 | 实测 | 期望 |
|----|------|------|
| V_d (Q10) | _____ | < 5000 (5V), 越接近 0 越好 |
| V_q (Q10) | _____ | ωe·ψf ≈ 750 (理论值) |
| Iq 跟踪误差 | _____ | < 200 |

---

### 阶段 1: BEMF FF 验证 (USE_WEAK_MAGN 暂时不触发)

#### T1.1 中速稳态 BEMF 检查
```
Runcmd2M3tar60        # 60rpm 输出端 = 1500rpm 电机端
等 5 秒
```
**判据**:
- [L200/3] BEMF_ff=1, Vq_ff ≈ 11~12V (= ωe×ψf)
- V_d 接近 0 (角度对齐)
- V_q ≈ Vq_ff + 小量 PI 残余

#### T1.2 反向切换测试 (BEMF FF 旧 bug 检测)
```
Runcmd2M3tar50
等 3 秒
Runcmd2M3tar-50       # 反向切换
观察 5 秒
```
**判据**:
- 切换瞬间无明显电流冲击
- 反向稳态 Vq_ff 应为负值
- 速度环 ramp 平滑过渡

**失败迹象**: 切换时 [L200/2] sat=1 持续 > 100ms, 或速度震荡 > 5rpm

---

### 阶段 2: 弱磁验证 (核心)

#### T2.1 触发阈值边界
```
Runcmd2M3tar110       # 110rpm 输出端, 接近基速
等 10 秒
```
**判据**:
- [L200/4b] vs_excess 在 ±100 范围内波动 (临界)
- id_weak 应在 0 附近抖动 (-300~0 量级)
- 速度跟随到 110rpm, 无震荡

#### T2.2 弱磁工作区
```
Runcmd2M3tar130       # 130rpm 输出端, 必触发
等 15 秒稳定
```
**判据**:
| 量 | 实测 | 期望 | 故障判定 |
|----|------|------|---------|
| 实际速度 | _____ | 接近 130rpm | 卡在 110~115 = 弱磁未生效 |
| id_weak (Q10) | _____ | -1000 ~ -3072 | 撞底 -3072 → 提高 ID_MIN |
| vs_excess | _____ | ≈ 0 (闭环稳态) | 持续>500 → PI 不稳 |
| Iq (Q10) | _____ | < iq_avail | sat=1 持续 → 增益过小 |
| Iq sat (L200/2) | _____ | 偶尔触发 | 持续=1 → anti-windup 异常 |
| sat_vs (L200/3) | _____ | 偶尔=1 | 持续=1 → 总电压撞顶, 弱磁太浅 |

#### T2.3 退出弱磁 (动态过程)
```
Runcmd2M3tar130       # 进入弱磁
等 5 秒
Runcmd2M3tar50        # 突减速
观察 5 秒
```
**判据**:
- id_weak 应在 100ms 内回到 0 (LEAK_OUT_STEP 速率)
- 无超速 (anti-windup 应阻止速度环积分饱和)
- 速度平滑下降到 50rpm

**失败迹象**: id_weak 卡在 -2000 不动, 或减速时超过 130rpm

#### T2.4 反向弱磁
```
Runcmd2M3tar-130      # 反向 130rpm
等 15 秒
```
**判据**:
- 与 T2.2 行为对称
- id_weak 仍是负值 (永磁体方向不变)
- vs_excess 触发 (用 |Vs| 判据, 不分方向)

#### T2.5 双向切换
```
Runcmd2M3tar130       # 正转弱磁
等 5 秒
Runcmd2M3tar-130      # 直接反向
观察 10 秒
```
**判据**:
- 切换瞬间无故障 (OverCurrent / OverSpeed / 缺相)
- id_weak 应在过零附近短暂归零, 反向稳定后再次 -2000~-3072

---

### 阶段 3: 缺相保护验证

#### T3.1 静止误触发检查
```
开机静止 60 秒
```
**判据**:
- 无 [14] PhaseU_Err / [15] PhaseV_Err / [16] PhaseW_Err 触发
- 三相电流自然漂移在 ±200 (Q10) 噪声内, sum < 1024 容差

#### T3.2 运行中三相平衡检查
```
Runcmd2M3tar50
等 30 秒
```
**判据**:
- 无缺相误触发
- [L200/4] |Ia| ≈ |Ib| ≈ |Ic| (三相对称)
- |Ia+Ib+Ic| < 1024 (KCL 满足)

#### T3.3 模拟缺相 (危险测试, 严格断电后操作)
```
注意: 必须断电后操作!

1. 断电
2. 拔下电机 U 相线
3. 上电
4. 触发运行: Runcmd2M3tar50
5. 等 200ms 内应触发 PhaseUVolErr
6. log 应有: FAULT! ServoErrFlag=0x???? PWM disabled
            [14] PhaseU_Err
```
**判据**:
- 触发时间 < 300ms (J2 单相低判据 100ms + 滤波)
- 故障后 `ServoState.Bit.ServoState_run` 应清零
- TIM1 BDTR.MOE 应为 0 (PWM 关闭)

#### T3.4 KCL 失衡测试 (无法实物模拟, 用代码注入)
跳过, 等真出问题再加 fault injection 接口

---

### 阶段 4: 综合稳定性

#### T4.1 长时间运行
```
Runcmd2M3tar100
观察 10 分钟
```
**判据**:
- 无故障打印
- id_weak 状态稳定 (有抖动是正常的, 因为 95% 阈值临界)
- MOS / 电机温度未触发 (T < 100°C)

#### T4.2 位置环兼容性
```
位置模式跑梯形规划:
Runcmd1M1tar18000     # 移动到 18000 (1°/1024 → ~17.6°)
等 3 秒
Runcmd1M1tar0
```
**判据**:
- 位置到达, 无超调
- 加减速过程 id_weak 跟随合理 (高速段触发, 低速段恢复)
- 位置环刚度无变化 (重构未破坏控制特性)

---

## 5. 故障应对预案

### 5.1 弱磁不触发
**现象**: 130rpm 时 id_weak 一直为 0
**排查**:
1. log 检查 `psi_f` 是否有效 (>0)
2. [L200/4b] 看 vs_excess 是否 >0 (可能因 BEMF FF 太强已经预先压住 Vs)
3. 提高 BEMF FF 软限幅系数 (0.85 → 0.75 留更多余量给弱磁)

### 5.2 弱磁震荡
**现象**: id_weak 大幅波动 (-3072 ↔ 0), 速度震荡
**排查**:
1. 减小 WMAG_KP / WMAG_KI (4/1 → 2/1)
2. 加深 LPF (α=1/16 → α=1/32)
3. 增加触发滞回 (95% → 92%)

### 5.3 弱磁退磁
**现象**: 关机后再开, 基速明显降低 (110→90rpm)
**排查**: ψf 永久变小, 重做 bwtest4 标定后看新值
**行动**: 立即 `WMAG_ID_MIN_Q10` 收紧到 -2048 (-2A)

### 5.4 缺相误触发
**现象**: 正常运行时 PhaseUVolErr 偶发
**排查**:
1. [L200/4] 观察故障前的三相电流值
2. 提高 `PhaseLossLowThresh` (307 → 500)
3. 增加 `PhaseLossFilterMs` (100 → 200)

### 5.5 BEMF FF 反向冲击
**现象**: 反向切换时电流尖峰 / 速度过冲
**排查**:
1. 增大 `MOTOR_DEAD_RPM_Q10` (30×1024 → 50×1024) 扩大过零带
2. 减小 `omega_e_filt` 滤波系数 (0.125 → 0.0625) 让前馈更滞后

---

## 6. 回归性指标对比表

| 工况 | 改前基线 | 改后预期 | 改后实测 | 判定 |
|------|----------|----------|----------|------|
| 50rpm 输出端稳态 V_d | ? | < 5V | _____ | _____ |
| 100rpm 输出端稳态 Iq | _____ | _____ (与改前同) | _____ | 应不变 |
| 100rpm 反向切换冲击 | 有 sat 持续 | 无 sat 持续 | _____ | _____ |
| 110rpm 弱磁触发率 | N/A | id_weak ≠ 0 | _____ | _____ |
| 130rpm 速度跟随 | 卡在 110 | 跟到 130 | _____ | _____ |
| ISR max 耗时 | ~58 µs | < 65 µs | _____ | _____ |

---

## 7. 验收标准

**通过条件 (全部满足才视为验证通过)**:

- [ ] 阶段 0 全部测试无故障误触发
- [ ] 阶段 1 BEMF FF 反向切换无明显冲击
- [ ] 阶段 2 弱磁能稳定到 130rpm 输出端 (扩 30%)
- [ ] 阶段 2 弱磁工作时 id_weak 不撞底 (-3072), 否则需放宽 ID_MIN
- [ ] 阶段 3 静止 + 运行 30 秒无缺相误触发
- [ ] 阶段 3 拔相测试在 300ms 内停机
- [ ] 阶段 4 长时间运行无故障 / 温升正常
- [ ] 位置环响应特性无回退 (与改前对比基线)
- [ ] ISR max 耗时增加 < 5µs

**不通过的应对**:
- 弱磁问题 → 调参 (5.1~5.3)
- 缺相误触发 → 调阈值 (5.4)
- BEMF FF 问题 → 调过零钳制 (5.5)
- 仍不通过 → 关闭对应功能 (`USE_WEAK_MAGN = 0` 等), 单独提交未启用版本

---

## 8. 后续工作 (本次不做)

- [ ] 弱磁双判据细化 (深度弱磁时启用三相不平衡判据)
- [ ] 温度补偿 (ψf 跟随电机温度漂移)
- [ ] 弱磁参数 Flash 持久化 + CAN 动态调参
- [ ] 缺相后定位具体相 (KCL 单独无法定位, 结合三相 RMS 比对)
- [ ] MTPA 联合 (当前电机凸极比 1.15 收益小, 优先级低)

---

## 9. 相关文档

- `CLAUDE.md` - 项目总览, 实测参数, 三环 PID 最佳值
- `HIGH_SPEED_100RPM_ANALYSIS.md` - 100rpm 工况扭矩分析
- `TORQUE_DEFICIT_VERIFY.md` - 33A 跑飞分析
- `FAULT_PROTECTION.md` - 故障保护层设计
- `过调制对策backlog.md` - 过调制对策
