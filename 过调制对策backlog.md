# 过调制 / 空载跑飞对策 Backlog

## 状态总览（截至 2026-05-14）

第一阶段全部落地，编译通过 0 Error 0 Warning，待烧录实测验证。

| 优先级 | 方案 | 状态 |
|--------|------|------|
| P0-1 | 速度环 `MIN_ACC_TIME` 200→400ms | ✅ 已改 |
| P1-3 | `INC_PID_SPEED_LIMIT` 25A→10A | ✅ 已改 |
| P1-2 | I_q_ref 直写路径限幅（CAN+串口）| ✅ 已加 |
| P0-2 | 速度环 PI 抗饱和 | ✅ 评估后跳过：IncPIDCal 输出钳位即天然抗饱和 |
| P0-3 | 反电动势前馈（BEMF FF）| ✅ 已加（`USE_BEMF_FF=1` 默认开）|
| P1-1 | 母线电压联动限流 | ⏸️ 待做 |
| P0-2b | 位置环 PositionPID 抗饱和 | ⏸️ 按需 |
| P3 | 过调制 SVPWM | ❌ 不做（48V 母线 + 100rpm 工况无需，反电动势仅 20V 占用）|

## 当前代码状况

| 方案 | 现状 | 位置 |
|------|------|------|
| 反电动势前馈 | ✅ **已实现** | `foc_current_loop.c` 电流 PI 后, `limit_norm` 前 |
| PI 抗饱和（速度环+电流环）| ✅ **天然抗饱和** | `IncPIDCal` 增量式 PID 自带 OutPut 钳位 |
| 速度环斜坡 | ✅ 已启用 `USE_SPEED_LOOP_SMOOTH=1` | `MIN_ACC_TIME=400ms`（加速度 250 rpm/s）|
| 电流指令限幅 | ✅ 三层都有 | 速度环 `INC_PID_SPEED_LIMIT=10A` + CAN/串口直写路径都加了钳位 |
| 母线电压联动限流 | ❌ 未实现 | VDC 已采样但只用于 OVP/UVP |
| 位置环 PositionPID 抗饱和 | ⚠️ 部分 | `pidI` 输出贡献钳位 ±20000，但 `iError` 累加无限制 |
| 过调制 SVPWM | ❌ 未实现（不需要）| 标准 SVPWM，圆形限幅到 V_dc/√3 ≈ 27.7V |

## 反电动势前馈实现

`foc_current_loop.c foc_current_close_loop` 内：

```c
#if USE_BEMF_FF
    if (controller->ident_test.flux_psi > 0.0f && controller->ident_test.Lq > 0.0f) {
        const float OMEGA_E_K = (float)NPP * 2.0f * 3.14159265f / (1024.0f * 60.0f);
        float omega_e = (float)controller->dtheta_mech * OMEGA_E_K;
        float Vd_ff = -omega_e * controller->ident_test.Lq * (float)controller->I_q;
        float Vq_ff =  omega_e * (controller->ident_test.Ld * (float)controller->I_d
                                  + controller->ident_test.flux_psi * 1024.0f);
        controller->V_d += (int32_t)Vd_ff;
        controller->V_q += (int32_t)Vq_ff;
    }
#endif
```

**单位推导**:
- `dtheta_mech` [电机端 rpm × 1024]
- `ωe_real` [rad/s 电] = `NPP × dtheta_mech / 1024 × 2π/60`
- `I_d/I_q` Q10 [1024=1A], `V_d/V_q` Q10 [1024=1V]
- `Vd_ff_q10 = -ωe·Lq·I_q`（1024 自抵消）
- `Vq_ff_q10 = ωe·(Ld·I_d + ψ_f×1024)`

**理论收益**（电机端 100 rpm = 输出端 100/25 = 4 rpm；如转输出端 100 rpm 则电机端 2500 rpm）：
- ωe = 2500 rpm 时 = 2094 rad/s（电）
- Vq_ff_real ≈ 2094 × 0.00967 = **20.25 V**（接管反电动势主体）
- Vd_ff_real ≈ -2094 × 0.113mH × 5A = **-1.18 V**（电感压降补偿）
- PI 残余只需补 Rs·I + 误差扰动 ≈ 2V，远低于过调制阈值

## 第二阶段（待做）

### P1-1 母线电压联动限流（半天）

**思路**: V_dc 低时调制度更易接近极限，动态降低 `INC_PID_SPEED_LIMIT` 给电流环留裕量。

```c
// 1ms 周期任务:
float vdc = get_vdc_volt();
float v_max = vdc / sqrtf(3.0f);
float v_actual = sqrtf(Vd*Vd + Vq*Vq) / 1024.0f;
float margin = v_max - v_actual;

if (margin < 2.0f) {
    INC_PID_SPEED_LIMIT_dynamic = 5 * 1024;   // 紧缩到 5A
} else if (margin < 5.0f) {
    INC_PID_SPEED_LIMIT_dynamic = 7 * 1024;
} else {
    INC_PID_SPEED_LIMIT_dynamic = 10 * 1024;  // 满量程
}
```

### P0-2b 位置环 PositionPID 抗饱和（按需）

**问题**: `pidI` 输出贡献钳位 ±20000，但 `iError` 自身无限累加。

**方案**: conditional integration（饱和反向时不积分）

```c
// PositionPID 内
if ((pidI_64 >= 20000 && error > 0) || (pidI_64 <= -20000 && error < 0)) {
    // 已饱和且误差同向, 不再累加
} else {
    pid->iError += error;
}
```

**优先级低**：位置环前有梯形规划+速度环斜坡，阶跃需求小。

---

## 验证计划（烧录后）

1. **空载阶跃响应**: `Runcmd2M3tar20480`（载端 20 rpm × 1024）
   - vofa+ 抓 `velocity_ref_filterd` / `dtheta_mech` / `I_q_ref` / `V_d` / `V_q`
   - 通过判据：超调 < 10%，电流峰值 < 10A，无 ServoErrFlag 触发
2. **回归 bwtest**:
   - `bwtest1` 电流环带宽（应保持 ~1700Hz @ Kp=45 Ki=4）
   - `bwtest2` 速度环带宽（应保持 ~45Hz @ Kp=1500 Ki=10）
   - **预期**：BEMF 前馈生效后，电流环波形更平稳，速度环超调降低
3. **CAN 联调**：
   - 关掉 `g_can_timeout_force_disable`
   - 通过 PCAN 跑 T1~T17 矩阵

## 引用

- `速度阶跃损坏风险分析.md` — 速度阶跃硬件损坏机理 + MIN_ACC_TIME 推导
- `FOC_ISR_OPTIMIZATION.md` — ISR 耗时分析（注意 BEMF 前馈每拍多 ~0.5μs）
- `FAULT_PROTECTION.md` — 故障保护层
- `CAN_WLY_PROTOCOL_VERIFY.md` — CAN 协议验证流程
