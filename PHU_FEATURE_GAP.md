# STM32 工程相对 PHU 参考工程的功能缺口盘点

对比基准：
- 工程 A（当前 STM32H743）：`cubemx_yxsui/`
- 工程 B（PHU / HPMicro 参考）：`hpm6e00evk_ifly_phu/hpm_apps/apps/foc/software/foc_app/src/`

核心差异文件：STM32 的 `foc/foc_app/ifly_fault.c` / `ifly_fault_api.c` / `ifly_led.c` / `ifly_test.c` 全为空 stub（函数体 `return 0` / 无动作），算法已移植但**外层任务层没接**。以下分模块列出。

## 1. 故障保护（ifly_fault）

| 项 | PHU 位置 | STM32 当前 | 优先级 |
|---|---|---|---|
| 母线过/欠压（OverUdc/LowUdc） | `ifly_fault.c:191-235 dcVoltageProFunc` 带 5 次滤波 | `ifly_fault.c:56 return 0`（空） | 高 — 无保护等于裸跑 |
| 板温过温（NTC 查表 + Warn/Err 两级） | `ifly_fault.c:244-270 boradTempProFunc` + `TemperatureInquiry` + `NTCTempTable` | 函数存在但空；STM32 ADC 规则通道有采 VDC/温度但没接进故障层 | 高 |
| 堵转（LockedRotorProFunc，扭矩/速度/位置三模式判据） | `ifly_fault.c:333-369` | 空 | 高 |
| 母线过流（busOverCurrentCheck） | `ifly_fault.c:378-393` | 空 | 高 |
| 驱动芯片 nFAULT 引脚 | `ifly_fault.c:402-418` 读 GPIO | 空；DRV8353 nFAULT 引脚未接入 | 高 — DRV 硬件故障无法感知 |
| 相电压欠压（UVW） | `ifly_fault.c:279-325 motorPhaseVolCheck` | 空 | 中 |
| 位置/速度/电流跟随偏差超限 | `ifly_fault.c:586-646 motorPos/Speed/CurrentOffsetCheck` | 空 | 中 |
| 软件位置限位（motorOverPosCheck） | `ifly_fault.c:655-663` | 空；`MaxPositionLimit` 已写 Flash 但未生效 | 中 |
| 双编码器一致性（Dual_Encoder_Fault_Detection） | `ifly_fault_api.c:146`，在 motorMonitor 里周期调 | `ifly_fault_api.c:146` 仅定义，无主循环调用 | 中 |
| 故障总分发 `CheckAndHandleAllFaultBits` + `CiA402_LocalError` | `ifly_fault.c:47-107` 映射 20+ ServoErrFlag 位 | `ifly_fault.c:24 return 0` | 高 |
| 故障阈值 Threshold 结构 + setter API (`set_Under_Udc_limit` 等) | `ifly_fault_api.c:20-35` 默认值 + 大量 set/get | STM32 侧 `ifly_fault_api.c` 也是 stub | 中 |

## 2. 状态指示（LED）

| 项 | PHU | STM32 | 优先级 |
|---|---|---|---|
| LED1/2/3 闪烁码 (`ledLightCode_Set1~5`) | `ifly_led.c:18-110` | 空 | 低 |
| `running_hint_led` / `fault_hint_led` / `ecat_hint_led` | `ifly_led.c:117-165` 按故障/硬件/警告级联 | 空 | 低 — 硬件只有 LED_RUN，直接嵌故障层即可 |
| `led_task1` 盖板后 LED3 灯语 FreeRTOS 任务 | `freertos_app.c:422-442` | 无 | 低 |

## 3. 对外接口层（模式切换 / 控制字）

PHU 的 `user/ciaToFocInterface.c`（274 行）是对象字典 ↔ FOC 的完整桥，覆盖：控制字/状态字（`motorStatusContrl` 刷新 `SERVICE_RUNNING_FLAG`，`ciaTargetReachUpdate` 刷 `TARGET_REACH_FLAG`，`ciaControlFaultReset` 处理 FAULTRESET 控制字，`sscCommunicationSolve` AL 状态机）、运行模式 0x6060/6061、位置窗口 0x6067/6068、速度/扭矩阈值、软限位、回零方法/速度/加速度（0x6098/6099/609A）、最大扭矩/电流/速度（0x6072/6080）、插值周期 0x60C2、前馈 0x202C、抱闸控制 0x2014、STO 0x2022/2024、历史故障 0x2021、ProfileVelocity/Acceleration/Deceleration（0x607F/6083/6084/6085）。

STM32：仅 `can_wly.c` 支持万里扬 FDCAN 从站，提供 **position / velocity / torque / MIT 模式**（`CAN_WLY_ID_*`），可切 `PROFILE_*_MODE` / `CYCLIC_SYNC_POSITION_MODE`。覆盖 CSP/CSV/CST 的控制信号层，但缺：故障复位控制字、位置到达状态字、STO、历史故障查询、软限位使能、回零、抱闸/刹车通道、前馈增益热配置。

优先级：高（CSP 已能跑）+ 中（状态字/故障复位不接会导致错了无法复位），若项目只走 CAN-FD 也要起码加故障复位和运行状态字。

## 4. 上电流程（对齐 / 抱闸 / 软启动）

| 项 | PHU | STM32 | 优先级 |
|---|---|---|---|
| 抱闸控制（GPIO 两路：delay / 主触点） | `ifly_fault.c:453-505 brake_open / brake_close / brake_close_limit` | 空 | **高，若硬件有抱闸** — 未知 STM32 硬件是否带抱闸；若无可忽略 |
| 抱闸上电延时 500ms 再刷 PDO | `ciaToFocInterface.c:120-133 brake_add_time` | 无 | 中 |
| 电角度偏置辨识 `set_elecoffest_action` + Flash 保存 | `ifly_fault_api.c:121`（API 挂钩），实际算法 `ElecAngleEstimate` | STM32 `foc_data.c` 有 `ElecAngleEstimate`，`Cali` 命令挂上 | 已覆盖 |
| ADC 电流零点校准 | PHU 初始化阶段 | STM32 `ADC_CalibrateOffsets(1024)` 已调 | 已覆盖 |
| 上电 LED3 常亮提示 | `freertos_app.c:167` `ledLightCode_Set4(LED3)` | 无 | 低 |
| 软启动/渐进使能 | PHU 靠抱闸延时 + 斜坡 | STM32 有 `SpeedLoopSmoothInit` 斜坡；抱闸链未接 | 中 |

## 5. 测试与诊断任务（ifly_test）

STM32 保留 `bwtest1~9` + 磁链/惯量/autoTune（覆盖 PHU 的 case 120/130/140/150/160）；**PHU 多出且 STM32 没补的**测试项（`ifly_test.c:51-158` 枚举）：

- case 1：`hpm_ota_soc_reset`（软复位 MCU）
- case 10/20/30：TestBusOverUdc / TestBusLowUdc / TestBoardOverTem（把阈值置 0 触发故障链，做故障流程回归）
- case 40/50/60：TestLockedRotor Current/Posit/Speed（堵转测试）
- case 70/80/90/100/110：TestSpeedOffset / TestMotorVelOver / TestPostionOver / TestIbusCurrentOver / TestUVWCurrentOver

这些测试全部依赖 "1 节" 的故障层，故障层补齐后自然补。优先级：低（验证用，不影响产品）。

另：STM32 的 `Test_log_print` 只处理带宽测试完成打印，没有 `testLogFlag` switch-case 调度（PHU 是 `ifly_test.c:51` 的大 switch）。

## 6. 算法（弱磁 / 无感 / HFI / 抱闸 / 凸极）

两边都没有弱磁 / HFI / PLL 无感 / 凸极 IPD。PHU 也不具备这些。不算缺口。

## 7. 调试命令与日志

STM32 `foc_bsp.c` `dbg_cmd_set`（`foc_bsp.c:138`）：logid / logfreq / CurrentPIDKpKiKd / SpeedPIDKpKiKd / PositionPIDKpKiKd / injectV / Cali / Runcmd / bwtest1~9。

PHU `foc_bsp.c:676 dbg_cmd_set` 额外支持：`logtest<N>`（触发 `ifly_test.c` 的故障/带宽测试），`dbg_log_print` 的 case 50/51/110/170/180/190/200~208/210/220 STM32 没有对应 logid（`dbg_log_print` STM32 只到 162）。

优先级：低 — `logtest` 依赖故障层；logid 200+ 多是 CiA402 对象字典/EtherCAT 状态调试，STM32 用不到。

STM32 `Test_log_print` 未轮询 `testLogFlag`（PHU 里是中央调度点）。建议要做故障回归时再补。

## 8. Flash 存储字段

PHU `FlashData` 结构预留 `temp1~temp8`（`foc_data.c:559-566` 默认 0），实际用途**未在 PHU 源码里明确绑定**。

STM32 已绑定：temp1=Rs、temp2=Ld、temp3=Lq、temp7=ψ_f、temp8=J，加三个 Flag（MotorParamFlag / FluxIdentFlag / InertiaIdentFlag）+ `FLASH_STRUCT_VERSION=3`，比 PHU 更完善。两边都有 `MaxPositionLimit / MinPositionLimit / PositionLimitFlag / Historical 错误队列`。

**缺口**：STM32 软限位字段已写 Flash，但 `motorOverPosCheck` 是空实现，**读了不用**。属第 1 节故障层附带。

## 9. 已知主动裁剪

- FreeRTOS：PHU 7 个任务（Init / my_task02 UART DMA / Debug / ota / motorMonitor(10ms) / RtosLog / led_task1）。STM32 裸跑 `main.c:205 while(1)` 里只调 `Test_log_print()`，**motorMonitor 的 10ms 周期故障扫描整块没搬**。这是第 1 节所有空函数的根因。补齐路线：main while 加 10ms tick（复用 TIM7 或 HAL_GetTick 轮询），调 `adc_convert` / `dcVoltageProFunc` / `LockedRotorProFunc` / `busOverCurrentCheck` / `driverChipFaultCheck` / `CheckAndHandleAllFaultBits` / `Dual_Encoder_Fault_Detection`。
- EtherCAT CiA402：已删 `cia402appl.h` 和 `ciaToFocInterface.c`。对外接口由 `can_wly.c` 覆盖，详见第 3 节。
- CANopen (canfestival)：PHU 源码里已注释，不算主动裁剪。
- `sei.c`：PHU 475 行，是 HPM SEI 外设驱动 BiSS-C 磁编；STM32 用 DPT RS485（`encoder.c`），不等价但功能对齐，`sei.c` STM32 保留 21 行空 stub 是编译兼容。不算缺口。
- `yxsui_test.c`：PHU 空文件（0 行），无实际代码。不算缺口。

## 总结与建议补齐优先级

高：故障层外层调度 + OVP/UVP/OC/OT/堵转/DRV nFAULT 引脚接入（第 1 节）。
中：CAN-FD 故障复位 + 状态字（第 3 节），软位置限位生效（第 1 节 motorOverPosCheck）。
低：LED 状态机、`logtest<N>` 故障回归链、抱闸（若无硬件则忽略）。

---

## 附录：motor_h7_0426 功能对比

对比基准：
- 工程 A（当前 STM32H743）：`cubemx_yxsui/`
- 工程 B（PHU 参考）：`hpm6e00evk_ifly_phu/`
- 工程 C（motor_h7_0426）：`90_product_260424/`

motor_h7 是同平台 STM32H7 的产品级参考工程，代码规模 1344 行（FOC + MotionControl 核心模块），主循环采用裸跑 + 1ms 周期任务轮询，故障保护在 1ms 任务中调用。以下列出 motor_h7 有但当前 STM32 缺失的功能，**粗体标记 PHU 也没有的 H7 平台特有功能**。

| 功能模块 | motor_h7 实现 | 当前 STM32 | PHU 有无 | 优先级 | 备注 |
|---------|--------------|-----------|---------|--------|------|
| **VOFA+ 上位机协议** | `uart4_VOFA+.c:1-80` DMA 发送浮点数组 + 尾帧 0x7F800000 | 未实现 | ✗ | 中 | motor_h7 通过 UART6 DMA 发送 19 通道浮点数据到 VOFA+，当前 STM32 仅有 printf 文本日志；VOFA+ 是工业级波形监控工具，比文本日志更高效 |
| **梯形 + PP 双轨迹规划器** | `trajectory.c:1-154` 梯形（数组预计算）+ `PPtraj.c:1-221` PP 模式（实时计算） | 部分实现 | ✗ | 中 | 当前 STM32 仅有 `PosTrapezoidPlan` 梯形规划；motor_h7 的 PP 模式支持三角/梯形自适应 + 初速度非零启动 + 到达判据（`bTargetPosFinish`），trajectory 模式支持 2000 步数组缓存 + 加减速分离配置 |
| **交互式菜单系统** | `interact.c:16-96` 串口菜单（m=电机模式/c=校准/s=设置/h=回零/e=编码器/z=零位） | 未实现 | ✗ | 低 | motor_h7 提供完整的人机交互界面，当前 STM32 仅支持 `logid<N>` 命令；菜单系统便于现场调试但非产品必需 |
| **状态机 FSMstate** | `FOC.c:31` REST/MOTOR/HOMING/CALIBRATE 四状态 + `main.c:265-278` 状态切换 | 未实现 | ✗ | 中 | motor_h7 用 FSMstate 管理运行模式切换，当前 STM32 直接用 `foc_run` 标志；状态机更清晰但当前架构够用 |
| **到达判据 bTargetPosFinish** | `main.c:230-244` 位置误差 < 0.015rad 持续 10ms 置位 | 未实现 | ✗ | 中 | motor_h7 提供位置到达标志供 CAN 上报，当前 STM32 缺此状态字；PHU 有 `TARGET_REACH_FLAG` 但未移植 |
| **CAN 主动上报模式** | `can_rv.c:18-25` CommResponseMode（询问应答）/ CommActiveMode（主动上报 100us/1ms） | 未实现 | ✗ | 中 | motor_h7 支持 100us 周期上报电流 + 1ms 周期上报全状态（位置/速度/扭矩/故障），当前 STM32 `can_wly.c` 仅被动应答 |
| **CAN 超时保护** | `can_rv.c:23-24` CAN_TIMEOUT 计数器，超时自动停机 | 未实现 | ✗ | 高 | motor_h7 在 1ms 任务中递减 CAN_timeout，归零时 `disablePWM()`；当前 STM32 无此保护，CAN 断线后电机继续执行旧指令 |
| **MIT 模式 KP/KD 动态范围映射** | `can_rv.c:4-13` KP_MIN/MAX、KD_MIN/MAX、POS_MIN/MAX 映射到 16bit CAN 字段 | 未实现 | ✗ | 低 | motor_h7 支持 MIT 模式的阻抗控制参数在线调节，当前 STM32 `can_wly.c` 有 MIT 模式但 KP/KD 固定 |
| **扫频测试 SweepSine** | `SweepSine.c:1-134` 线性扫频发生器（0→f_max，可配振幅/扫描时间） | 未实现 | ✗ | 低 | motor_h7 用于电流环带宽测试（`main.c:187-191` 初始化 2.43A/1500Hz/10s），当前 STM32 `bwtest1` 用固定频率点注入；扫频更精细但当前方案够用 |
| 故障保护（1ms 周期） | `Diag.c:34-68` OverCurrentPro（3 点滑动均值 + 10 次计数）<br>`Diag.c:69-113` VoltErr（OVP 60V / UVP 24V）<br>`Diag.c:114-160` MosTempErr（90°C 警告 / 100°C 停机）<br>`Diag.c:161-207` MotorTempErr（NTC 查表） | 部分实现 | ✓ | 高 | 当前 STM32 `ifly_fault.c` 是空 stub；motor_h7 在 `main.c:258-260` 1ms 任务中调用三个保护函数，PHU 也有类似实现但在 10ms 任务；**motor_h7 的 NTC 温度查表公式与 PHU 不同**（B 常数 3950 vs 3435） |
| 编码器超速保护 | `Diag.c:25-32` EncoderErr（速度超 w_max/w_min 停机） | 未实现 | ✓ | 中 | PHU 有 `motorSpeedOffsetCheck`，当前 STM32 未接入 |
| **电流 RMS 计算** | `FOC.c:未找到具体实现，但 main.c:226 调用 Calc_current_rms()` | 未实现 | ✗ | 低 | motor_h7 计算相电流有效值用于过载保护，当前 STM32 仅采样瞬时值 |
| **温度采样与滤波** | `motor.c:94-99` NTC 阻值→温度转换 + 二阶滤波初始化<br>`main.c:227` temperatureSample() 1ms 周期更新 | 未实现 | ✓ | 高 | 当前 STM32 ADC 规则通道采 VDC/温度但未处理；motor_h7 用 B 常数公式（MOS 3950K / 电机 3435K）+ 滑动均值；PHU 有 NTCTempTable 查表 |
| PID 结构差异 | `pid.c:43-55` 支持 deadband（死区）+ feedforward_ratio（前馈比例） | 未实现 | ✗ | 低 | 当前 STM32 `func_pid.c` 是增量式 PID，motor_h7 是位置式 PID + 死区 + 前馈；两种实现各有优劣，当前架构够用 |
| **Flash 字段差异** | `flash.c:47-81` 存 32 字段：elec_offset / phase_order / cali_finish / Rs / Ld / motor_calibrated / mech_offset / p_min/max / w_min/max / iq_min/max / KP_MIN/MAX / KD_MIN/MAX / FDCAN_ID | 部分实现 | ✗ | 中 | 当前 STM32 存 Rs/Ld/Lq/ψ_f/J + 软限位；motor_h7 多存 **CAN ID、MIT 模式范围、机械零位偏置**；当前缺 CAN ID 热配置（需重新编译） |
| **校准流程 order_phases** | `calibration.c:16-70` 相序检测（正转 4π 电角度判断编码器与电机同向/反向） | 未实现 | ✗ | 低 | 当前 STM32 `ElecAngleEstimate` 仅做电角度偏置辨识，motor_h7 额外检测相序；若硬件接线固定可省略 |
| **校准流程 LUT 非线性校正** | `calibration.c:5-15` 预留 error_f/error_b/offset_lut 数组（代码中已注释） | 未实现 | ✗ | 低 | motor_h7 预留编码器非线性校正（128 点 LUT），当前未启用；高端编码器不需要 |
| **电阻/电感辨识（电机层）** | `motor.c:未完整实现，仅有 MeasureResistance/MeasureInductance 函数声明` | 已覆盖 | ✓ | - | 当前 STM32 `ifly_flux_ident.c` 已实现 Rs/Ld/Lq 辨识，motor_h7 此部分未完成 |
| SVPWM 过调制 | `FOC.c:45` OVERMODULATION 1.15 系数 | 未实现 | ✗ | 低 | motor_h7 支持 15% 过调制提升电压利用率，当前 STM32 限幅在 Vbus/√3；过调制会增加谐波，需权衡 |
| **主循环 1ms 任务** | `main.c:224-262` 1ms 标志轮询：Calc_current_rms / temperatureSample / bTargetPosFinish 判断 / VOFA 发送 / VoltErr / MosTempErr / MotorTempErr | 未实现 | ✗ | 高 | 当前 STM32 主循环仅调 `Test_log_print()`；motor_h7 的 1ms 任务是故障保护 + 状态监控的核心调度点，**这是 motor_h7 与 PHU（10ms 任务）的关键差异** |
| **100us 任务（CAN 上报）** | `main.c:216-222` 100us 标志轮询：Pack_ActiveReport_Current + CAN_SendMessage | 未实现 | ✗ | 中 | motor_h7 支持 10kHz 电流上报（响应式控制），当前 STM32 无此高频通信 |
| **扭矩精度测试** | `main.c:205-214` TorqueTestFlag 触发 0→24.32A 分 10 档阶跃（每档 5s） | 未实现 | ✗ | 低 | motor_h7 的自动化测试工具，当前 STM32 可用 `Runcmd` 手动测 |
| **DWT 时间戳（ISR 耗时）** | `FOC.c:36-37` ISR_start/ISR_end + ISR_time_us 计算 | 已覆盖 | ✓ | - | 当前 STM32 `g_adc_isr_cycles` 已实现，motor_h7 用 DWT 计算 us（两者等价） |
| **故障码 Err1/Err2/Warning** | `motor.c:83-85` 三级故障分类（Err1=致命 / Err2=次要 / Warning=警告） | 未实现 | ✓ | 中 | PHU 有 `ServoErrFlag` 20+ 位，当前 STM32 未分级；motor_h7 的三级分类便于 CAN 上报 |
| **电流补偿系数** | `motor.c:55` current_compensation_ratio（代码中未定义具体值） | 未实现 | ✗ | 低 | motor_h7 预留电流传感器非线性补偿，当前 STM32 用线性公式 |

### motor_h7 平台特有功能清单（PHU 无）

以下 15 项功能 motor_h7 有但 PHU 也没有，属于 STM32H7 平台特有或 motor_h7 团队独立开发：

1. **VOFA+ 上位机协议**（uart4_VOFA+.c）— 工业级波形监控，比 printf 高效
2. **PP 模式轨迹规划**（PPtraj.c）— 实时计算三角/梯形 + 初速度非零
3. **交互式菜单系统**（interact.c）— 串口人机界面
4. **状态机 FSMstate**（FOC.c）— REST/MOTOR/HOMING/CALIBRATE 四状态管理
5. **到达判据 bTargetPosFinish**（main.c）— 位置到达标志
6. **CAN 主动上报模式**（can_rv.c）— 100us 电流 + 1ms 全状态周期上报
7. **CAN 超时保护**（can_rv.c）— 断线自动停机
8. **MIT 模式 KP/KD 动态映射**（can_rv.c）— 阻抗参数在线调节
9. **扫频测试 SweepSine**（SweepSine.c）— 线性扫频发生器
10. **电流 RMS 计算**（FOC.c）— 相电流有效值
11. **PID 死区 + 前馈**（pid.c）— 位置式 PID 扩展
12. **Flash 存 CAN ID + MIT 范围**（flash.c）— 热配置支持
13. **相序检测 order_phases**（calibration.c）— 编码器方向自动判断
14. **SVPWM 过调制**（FOC.c）— 15% 电压利用率提升
15. **主循环 1ms 任务**（main.c）— 故障保护 + 监控调度（PHU 是 10ms）

### 补齐建议

**高优先级**（产品级必需）：
- CAN 超时保护（断线停机）
- 1ms 周期任务（故障保护调度）
- 温度采样与 NTC 转换
- 故障码三级分类（Err1/Err2/Warning）

**中优先级**（增强体验）：
- VOFA+ 协议（替代 printf 文本日志）
- CAN 主动上报模式（100us 电流 + 1ms 状态）
- 到达判据 bTargetPosFinish
- Flash 存 CAN ID（避免重新编译）
- PP 模式轨迹规划（若需初速度非零启动）

**低优先级**（锦上添花）：
- 交互式菜单（现场调试便利）
- 扫频测试（当前 bwtest1 够用）
- 电流 RMS / 过调制 / 相序检测（性能优化）

**已覆盖无需补**：
- Rs/Ld/Lq 辨识（当前 STM32 更完善）
- DWT 时间戳（已实现）
- 编码器超速保护（PHU 有，待接入）
