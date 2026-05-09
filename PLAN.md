# STM32H7 FOC 项目开发计划

**最后更新**: 2026-05-09
**当前状态**: 电流环闭环调通 + 4 项辨识完成

---

## 一、已完成功能

### 1. 电机参数辨识链（开机自动）

| 参数 | 实测值 | 手册值 | 误差 | 配置 |
|------|--------|--------|------|------|
| Rs | 0.0764 Ω | 0.0701 Ω | +9% (寄生+死区) | `measurePhaseResistance` |
| Ld | 0.113 mH | — | — | `measurePhaseInductanceAC` @ 500Hz |
| Lq | 0.113 mH | ~L_LL/2=0.107mH | -6% | SPM 非凸极 |
| elec_offest_0/1 | 1527/23484 | — | — | `ElecAngleEstimate` |

**相关文件**:
- `foc/foc_fast/foc_api.c` — Rs/Ld/Lq 辨识 + autoTune
- `foc/foc_fast/foc_data.c` — ElecAngleEstimate
- `foc/foc_fast/foc_bsp.c` — `Cali` 命令

### 2. 电角度补偿（已移植 motor_h7 的延迟补偿）

- 公式: `theta_comp = dtheta * 0.3` (对应 30µs 延迟)
- 位置: `foc/foc_fast/encoder_calc.c` Line 70~95
- 效果: 1667rpm 下补偿 2.4° 相位超前

### 3. 死区补偿（已禁用）

- 宏: `USE_DEADTIME_COMPENSATION 0`
- 原因: bang-bang 切换在 251Hz 制造 +4dB 伪谐振峰
- 长期方案: 改软切换 (tanh 过渡带) 后可重新启用

### 4. 电流环带宽测试

- 命令: `bwtest1`
- 范围: 10~2500Hz 扫频
- 输出: Bode 表 + 性能指标汇总（峰值/带宽/ζ/PM/超调）
- 相关文件: `foc/foc_fast/foc_current_loop.c` (bw_test_*)

### 5. 最佳电流环配置（实测验证）

| 参数 | 值 |
|------|-----|
| Kp | 45 |
| Ki | 4 |
| PID_Div | 100 |
| TargetBW | 800Hz (需降到 650Hz 让 autoTune 自动给出 Kp=46) |

**性能指标**（Kp=45 实测）:
- 带宽: 1710 Hz ✅
- 峰值: 0.86 dB ✅
- 阻尼比: 0.54 ✅
- 相位裕度: 54° ✅
- 超调: 14% ✅

---

## 二、待办任务（按优先级）

### 🟢 优先级 1: 速度环启转验证（半天）

**目标**: 验证速度闭环能稳定运行到目标转速

**步骤**:
1. 串口发送 `Run cmd1 M3 tar100` (velocity mode, 100rpm)
2. 观察 `logid 50` 速度跟踪情况
3. 调整速度环 PI (当前 Kp=2000 Ki=10，可能偏保守)

**前置条件**:
- ✅ 电流环稳定
- ✅ 电角度偏置已标定
- ✅ 速度采样工作正常

**风险点**:
- 速度环 PI 参数不合适 → 超调/振荡
- 减速比 50:1 带来的反电动势限幅

---

### 🟢 优先级 2: 速度环带宽测试（半天）

**目标**: 扫频测量速度环 Bode，评估稳定性

**工作量**: 底层已完成，只需补外层

**需要实现**:
```c
// foc/foc_app/ifly_test.c
void TestSpeedLoopBandwidth(void) {
  controller_eyou.controller_mode = TEST_MOTOR_SPEED_MODE;
  controller_eyou.velocity_ref = 10 * 1024 * 101;  // 偏置 10rpm
  controller_eyou.foc_run = 1;
  spd_bw_test_init(&controller_eyou.spd_bw_test, 1.0f, 200.0f,
                   2.0f * 1024 * 101, 5.0f);
  controller_eyou.spd_bw_test.saved_velocity_coe = Threshold.velocity_coe;
  Threshold.velocity_coe = 100;  // 临时放宽保护
  printf("Speed BW test started\r\n");
}
```

**命令扩展** (`foc_bsp.c` dbg_cmd_set):
```c
if (which == 2) TestSpeedLoopBandwidth();
```

**Test_log_print 里加**:
```c
if (controller_eyou.spd_bw_test.done) {
  spd_bw_test_print_results(&controller_eyou.spd_bw_test);
  controller_eyou.spd_bw_test.done = 0;
}
```

**参考**: `hpm6e00evk_ifly_phu/.../ifly_test.c:392`

---

### 🟡 优先级 3: 磁链辨识 (ψ_f)（1 天）

**目标**: 辨识永磁磁链，为惯量辨识和速度环 autoTune 准备

**原理**: 在两个 I_d 工况下稳态运行，测 V_q/I_q/I_d/ω_e
```
ψ_f = ((u_q1 - R_s·i_q1)·i_d2 - (u_q2 - R_s·i_q2)·i_d1) / (ω_e·(i_d2 - i_d1))
```

**工作量**:
- 核心 `runFluxIdent()` 已移植到 `foc/foc_app/ifly_flux_ident.c`
- 需要实现外层 `TestFluxIdent()`
- 需要 stub 几个辅助函数:
  - `motorStopProgress(mod)` — 空实现（无抱闸）
  - `brake_open(timeout)` — 空实现
  - `motorErrReset()` — 已有

**命令**: `bwtest4` 或 `logtest140`

**参考**: `hpm6e00evk_ifly_phu/.../ifly_test.c:415`

**前置条件**:
- ✅ Rs 已知（0.0764Ω）
- ⚠ 速度闭环能稳定 (依赖任务 1)
- ⚠ 需要机械能自由旋转（不能被锁住）

---

### 🟡 优先级 4: 位置环带宽测试（半天）

**目标**: 扫频测位置环 Bode，评估整链动态

**工作量**: 类似任务 2
- 底层 `pos_bw_test_*` 已完成
- 补 `TestPositionLoopBandwidth()` 外层

**注意**:
- 位置环测试需要电机空载自由旋转
- 注入幅值小（2° 以内）避免机械限位
- 扫频范围 1~100Hz

**参考**: `hpm6e00evk_ifly_phu/.../ifly_test.c:572`

---

### 🟠 优先级 5: 惯量辨识 (J)（1 天）

**目标**: 辨识转动惯量，用于速度环 autoTune

**原理**: 转矩模式 ±I_q 阶跃 + 速度门限触发反转
```
J = (mean(Te+) - mean(Te-)) / (mean(α+) - mean(α-))
```

**前置条件**:
- ⚠ 必须先跑磁链辨识（任务 3）拿到 ψ_f
- ✅ Ld, Lq 已知
- ⚠ 电机能做 ±I_q 阶跃（不能被锁住）

**工作量**:
- 核心 `runInertiaIdent()` 已移植到 `foc/foc_app/ifly_inertia_ident.c`
- 需实现外层 `TestInertiaIdent()`

**命令**: `bwtest5` 或 `logtest150`

**参考**: `hpm6e00evk_ifly_phu/.../ifly_test.c:477`

---

### 🔵 优先级 6: 速度环 autoTune（跟随任务 5 后）

**目标**: 自动整定速度环 PI

**现状**: `autoTuneSpeedLoopPI(J, psi_f, NPP)` 已实现在 `foc_api.c:462`

**触发**: 拿到 J 和 ψ_f 后自动调用，或串口命令手动触发

**公式**:
```
Kt = 1.5 × Pp × ψ_f
ω_c = 2π × SPEED_LOOP_TARGET_BW_HZ (60Hz)
Kp_w = J × ω_c / Kt
Ki_w = Kp_w × ω_c / 8  (零点滞后因子 8)
```

### 🔵 优先级 6.5: 位置环 PI 手动整定（跟随任务 4 后）

**目标**: 基于位置环 bwtest 结果手动调整 PI 参数

**说明**: **PHU 和 motor_h7 都没有位置环 autoTune**
- 位置环对机械参数（减速箱刚度、负载惯量、摩擦）高度敏感
- 工业实践都是手动整定 + 实验验证
- 当前 id=90 配置 `Kp=30000 Ki=1000` 已是合理起点

**经验公式**（如果要加 autoTune）:
```c
// 位置环带宽 ≈ 速度环带宽 / 4 = 60/4 = 15Hz
// 采样率 = FOC_FREQ / POSITION_CALCULATE_DIV = 2500Hz
float omega_c = 2π × 15 = 94 rad/s;
Kp_pos = omega_c × POSITION_PID_DIV = 94 × 100 = 9400;
Ki_pos = Kp_pos × (omega_c/10) × Ts = 35;
```

**建议流程**:
1. 先跑 bwtest3 实测位置环
2. 看峰值/带宽/相位裕度是否合格
3. 不合格再调 Kp（调 Ki 影响小）

---

### 🟣 优先级 7: CAN 通信协议从站验证（半天~1 天）

**现状**: 代码集成完成（上次提交），尚未实测

**相关文件**:
- `Core/Src/fdcan.c` — 底层 send/recv
- `Core/Src/can_wly.c` — 万里扬 V1.7 协议从站
- `Core/Inc/can_wly.h`
- `cubemx_yxsui.ioc` — CAN 配置

**验证步骤**:
1. 连 USB-CAN 工具
2. 发送万里扬 V1.7 标准帧
3. 观察响应帧是否符合协议
4. 1ms tick 主动上报是否正常

---

## 三、死区补偿修复（可选）

**当前问题**: bang-bang 切换在 251Hz 制造 +4dB 伪谐振峰

**修复方案**:

### 方案 A: 软切换
```c
// foc_current_loop.c deadtime_compensation()
#define DEADTIME_TRANSITION_BAND 256  // 0.25A
float ratio = I_phase / DEADTIME_TRANSITION_BAND;
if (ratio > 1.0f) ratio = 1.0f;
if (ratio < -1.0f) ratio = -1.0f;
V_comp = ratio * DEADTIME_COMP_VOLTAGE * 0.6f;
```

### 方案 B: 提高阈值
```c
#define DEADTIME_CURRENT_THRESHOLD 1536  // 1.5A
#define DEADTIME_COMP_VOLTAGE 180
```

**验证**: 开启死区补偿后重跑 `bwtest1`，峰值应 <2dB

---

## 四、长期优化（有时间再做）

### 1. 提高 FOC 频率到 20kHz
- ZOH 延迟减半 (150µs → 75µs)
- 电流环带宽上限翻倍
- CPU 负载: 40% → 80%
- 需验证 ADC 采样时间足够

### 2. PID_Div 调大到 1000
- autoTune Ki 量化精度从 25% 提升到 2.5%
- Kp/Ki 同步放大 10 倍
- 所有调用点都需同步修改

### 3. 弱磁控制
- 当前 `USE_WEAK_MAGN = 0`
- 需要先完成 Flux/Inertia 辨识
- 扩展工作转速（超过反电动势限制）

---

## 五、参考路线图

```
【已完成】
  电流环辨识 + autoTune + bwtest1 验证 ✓
        ↓
【下一步】(优先级 1)
  Run cmd1 M3 tar100 → 速度闭环启转 ← 现在做这个
        ↓
(优先级 2) 速度环 bwtest2
        ↓
(优先级 3) 磁链辨识 ψ_f
        ↓
(优先级 5) 惯量辨识 J
        ↓
(优先级 6) 速度环 autoTune
        ↓
(优先级 4) 位置环 bwtest3
        ↓
(优先级 7) CAN 通信实测
```

---

## 六、关键文件索引

| 文件 | 作用 |
|------|------|
| `foc/foc_fast/foc_api.c` | Rs/Ld/Lq/ElecAngle 辨识 + PI autoTune |
| `foc/foc_fast/foc_current_loop.c` | 电流环 + 死区补偿 + bw_test |
| `foc/foc_fast/foc_speed_loop.c` | 速度环 + spd_bw_test |
| `foc/foc_fast/foc_position_loop.c` | 位置环 + pos_bw_test |
| `foc/foc_fast/foc_data.c` | Flash 参数存储 + ElecAngleEstimate |
| `foc/foc_fast/foc_bsp.c` | 串口命令解析 + 日志 |
| `foc/foc_fast/encoder_calc.c` | 编码器解析 + theta 补偿 |
| `foc/foc_app/ifly_flux_ident.c` | 磁链辨识核心 (runFluxIdent) |
| `foc/foc_app/ifly_inertia_ident.c` | 惯量辨识核心 (runInertiaIdent) |
| `foc/foc_app/ifly_test.c` | 测试任务编排（当前大部分为空实现） |
| `Core/Src/main.c` | 上电初始化流程 |

---

## 七、参考项目

- **PHU (主要参考)**: `C:\Users\syx19\Desktop\src_git\hpm6e00evk_ifly_phu`
  - 已移植: 辨识流程、Cali、bw_test 底层
  - 待参考: flux/inertia 任务编排、速度/位置 bw_test 外层

- **motor_h7 (辅助参考)**: `C:\Users\syx19\Desktop\src_git\motor_h7`
  - 已移植: 编码器延迟补偿
  - 单位换算参考: 16位 ADC + 运放 + 采样电阻

---

## 八、性能目标 (最终验收)

| 环路 | 目标带宽 | 峰值 | 相位裕度 |
|------|---------|------|---------|
| 电流环 | 1500~2000 Hz | <3dB | >40° |
| 速度环 | 50~100 Hz | <3dB | >40° |
| 位置环 | 20~50 Hz | <3dB | >40° |

**整机性能指标**:
- 空载最高转速: 2000+ rpm (输出端 40+ rpm)
- 定位精度: ±0.1° (输出端，24位编码器极限)
- 转矩响应时间: <10ms (0→额定)
