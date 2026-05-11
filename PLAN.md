# STM32H7 FOC 项目开发计划

**最后更新**: 2026-05-11
**当前状态**: 三环 PID 全部跑通 + 完整辨识链 + Flash 缓存 + autoTune

---

## 一、已完成（本阶段）

### 1. 电机参数辨识链（已落盘 Flash）

| 参数 | 实测值 | 手册/理论 | 误差 | 入口命令 |
|------|--------|----------|------|---------|
| Rs | 0.0764 Ω | 0.0701 Ω | +9% (寄生+死区) | `bwtest3` |
| Ld | 0.113 mH | — | — | `bwtest3` |
| Lq | 0.113 mH | ~L_LL/2=0.107mH | -6% (SPM 非凸极) | `bwtest3` |
| ψ_f | 0.00967 Wb | — | — | `bwtest4` |
| J | 1.22e-4 kg·m² | — | — | `bwtest5` |
| elec_offest_0/1 | 1527/23484 | — | — | `Cali` |

**机制**:
- `identifyMotorParamsCached()` 上电检查 `MotorParamFlag`，命中则直接加载，无则辨识 + 写 Flash
- Flash 字段：`temp1=Rs, temp2=Ld, temp3=Lq, temp7=ψ_f, temp8=J`
- 三个独立 Flag：`MotorParamFlag` / `FluxIdentFlag` / `InertiaIdentFlag`
- 版本不匹配（v2→v3）强制全部重新辨识

### 2. 三环 PID 实测最佳

| 环路 | Kp | Ki | PID_Div | 实测 BW | 谐振峰 | 推荐 |
|------|-----|-----|---------|--------|--------|------|
| 电流环 | 45 | 4 | 100 | 1710Hz | +0.86dB | ✅ |
| 速度环 | 1500 | 10 | 65000 | 45.7Hz | +1.8dB | ✅ |
| 位置环 | 3016 | 9 | 100 | 25.6Hz | +0.58dB | ✅ |

### 3. autoTune 三环（IMC + 经验系数）

| 入口 | SAFETY_FACTOR | 目标 BW | 公式 |
|------|---------------|---------|------|
| `bwtest6` (电流环) | 1.0 | 650Hz | `Kp = ωc·Lq·Div`, `Ki = ωc·Rs·Div·Ts` |
| `bwtest7` (速度环) | 0.6 | 60Hz | `Kp = ωc·J/Kt · 0.6`, `Ki = Kp·ωc/8 · Ts` |
| `bwtest8` (位置环) | 0.4 | 12Hz | 经验公式，独立于电机参数 |

**SAFETY_FACTOR 来历**:
- 电流环 1.0：电流环带宽远高于反电动势/编码器滤波，IMC 模型准
- 速度环 0.6：补偿电流环 60Hz 处相位滞后 + 编码器滤波 12° + 减速箱
- 位置环 0.4：补偿机械刚度有限 + 减速箱反向间隙；实测 0.6 撞速度环饱和发散

### 4. 完整带宽测试链

| 命令 | 功能 | 范围 | 注入 |
|------|------|------|------|
| `bwtest1` | 电流环 Bode | 10~2500Hz | 0.3A 偏置 0.5A |
| `bwtest2` | 速度环 Bode | 1~200Hz | 2rpm 偏置 10rpm |
| `bwtest9` | 位置环 Bode | 4~100Hz | 2° 静态参考 (CSP 模式) |

### 5. 电角度补偿（已移植 motor_h7）
- 公式: `theta_comp = dtheta * 0.3` (对应 30µs 延迟)
- 位置: `foc/foc_fast/encoder_calc.c`
- 效果: 1667rpm 下补偿 2.4° 相位超前

### 6. 位置环梯形规划修复
- `VMAX = 100 output rpm` (对齐 MaxSpeed 限幅)
- `AMAX = 230 output rpm/s` (~0.5s 加速到位)
- **已对齐判断带 5 LSB (≈0.005°) 死区**: 避免 snap → 重启 → 反向减速的极限环

### 7. 死区补偿（暂禁用）
- 宏: `USE_DEADTIME_COMPENSATION 0`
- 原因: bang-bang 切换在 251Hz 制造 +4dB 伪谐振峰
- 长期方案: 改软切换 (tanh 过渡带)

---

## 二、待办任务（按优先级）

### 🟢 优先级 1: FDCAN 通信实测（半天~1天）

**现状**: 代码集成完成，未实测

**相关文件**:
- `Core/Src/fdcan.c` — 底层 send/recv
- `Core/Src/can_wly.c` — 万里扬 V1.7 协议从站
- `cubemx_yxsui.ioc` — CAN 配置

**验证步骤**:
1. 连 USB-CAN 工具
2. 发送万里扬 V1.7 标准帧
3. 观察响应帧是否符合协议
4. 1ms tick 主动上报是否正常

---

### 🟡 优先级 2: ADC ISR 瘦身（重要）

**现状**: ADC ISR 占用 60% 周期（稳态 42μs，速度环参与拍 58~60μs）

**影响**:
- 未来扩展功能（弱磁、更高控制频率）受限
- 编码器 DMA 完成中断被 ADC ISR 抢占，时间戳记录被延迟

**优化方向**:
- 把速度环（5kHz）从 ADC ISR 中移出，放到主循环 + flag 调度
- 位置环（2.5kHz）同样移到主循环
- ADC ISR 只保留：电流采样 + Clarke/Park + 电流 PID + SVPWM（目标 ~25μs）

**风险**:
- 移出后速度环延迟一拍（200μs），需评估对 BW 影响

---

### 🟡 优先级 3: 死区补偿修复（可选）

**当前问题**: bang-bang 切换在 251Hz 制造 +4dB 伪谐振峰

**修复方案 A: 软切换**
```c
// foc_current_loop.c deadtime_compensation()
#define DEADTIME_TRANSITION_BAND 256  // 0.25A
float ratio = I_phase / DEADTIME_TRANSITION_BAND;
if (ratio > 1.0f) ratio = 1.0f;
if (ratio < -1.0f) ratio = -1.0f;
V_comp = ratio * DEADTIME_COMP_VOLTAGE * 0.6f;
```

**修复方案 B: 提高阈值**
```c
#define DEADTIME_CURRENT_THRESHOLD 1536  // 1.5A
```

**验证**: 开启死区补偿后重跑 `bwtest1`，峰值应 <2dB

---

### 🔵 优先级 4: 长期优化

#### a. 提高 FOC 频率到 20kHz
- ZOH 延迟减半 (150µs → 75µs)
- 电流环带宽上限翻倍
- CPU 负载: 60% → 120% → **必须配合 ADC ISR 瘦身**
- 需验证 ADC 采样时间足够

#### b. PID_Div 调大到 1000
- autoTune Ki 量化精度从 25% 提升到 2.5%
- Kp/Ki 同步放大 10 倍
- 所有调用点都需同步修改

#### c. 弱磁控制
- 当前 `USE_WEAK_MAGN = 0`
- 已有 Flux/Inertia 辨识结果
- 扩展工作转速（超过反电动势限制）

---

## 三、路线图

```
【已完成】
   电流环辨识 + autoTune + bwtest1 ✓
        ↓
   速度环 bwtest2 + 调参 ✓
        ↓
   bwtest3 (Rs/Ld/Lq) → Flash 缓存 ✓
        ↓
   bwtest4 (ψ_f) → Flash ✓
        ↓
   bwtest5 (J) → Flash ✓
        ↓
   bwtest6/7/8 三环 autoTune + SAFETY_FACTOR 调优 ✓
        ↓
   bwtest9 位置环 BW + 梯形规划修复 ✓
        ↓
【下一步】
   FDCAN 通信实测 ← 优先级 1
        ↓
   ADC ISR 瘦身（速度/位置环移出） ← 优先级 2
        ↓
   死区补偿软切换 + 提速到 20kHz + 弱磁
```

---

## 四、关键文件索引

| 文件 | 作用 |
|------|------|
| `Core/Src/main.c` | 上电初始化主流程 |
| `foc/foc_fast/foc_api.c` | Init_foc / 辨识 / autoTune / Cached 加载 |
| `foc/foc_fast/foc_data.c` | Flash 数据管理 + ElecAngleEstimate |
| `foc/foc_fast/foc_controller.h` | FlashSavedData 结构（含 v3 新增 Flag） |
| `foc/foc_fast/foc_current_loop.c` | 电流环 + 死区补偿 + bw_test |
| `foc/foc_fast/foc_speed_loop.c` | 速度环 + spd_bw_test |
| `foc/foc_fast/foc_position_loop.c` | 位置环 + 梯形规划 + pos_bw_test |
| `foc/foc_fast/foc_bsp.c` | 串口命令解析 (bwtest1~9) + 日志 |
| `foc/foc_fast/encoder_calc.c` | 编码器解析 + theta 补偿 |
| `foc/foc_app/ifly_flux_ident.c` | 磁链辨识核心 |
| `foc/foc_app/ifly_inertia_ident.c` | 惯量辨识核心 |
| `foc/foc_app/ifly_test.c` | 测试任务编排 (Test* 函数) |

---

## 五、参考工程

- **PHU**: `C:\Users\syx19\Desktop\src_git\hpm6e00evk_ifly_phu`
  - 主要参考：完整 FOC 架构、辨识流程、带宽测试底层
- **motor_h7**: `C:\Users\syx19\Desktop\src_git\motor_h7`
  - STM32H7 + 小电机硬件参数
  - 编码器延迟补偿算法

---

## 六、性能目标（验收）

| 环路 | 设计带宽 | 实测带宽 | 状态 |
|------|---------|---------|------|
| 电流环 | 650~800 Hz | **1710 Hz** | ✅ 超额 |
| 速度环 | 60 Hz | **45.7 Hz** | ✅ 接近 |
| 位置环 | 12 Hz | **25.6 Hz** | ✅ 超额 |

| 整机指标 | 当前 | 目标 |
|---------|------|------|
| 空载最高转速 | 载端 100rpm = 电机端 2500rpm | 一致 |
| 定位精度 | ±0.005° (输出端，5 LSB 死区) | ±0.1° |
| 转矩响应时间 | <5ms (电流环 BW 1710Hz) | <10ms |
