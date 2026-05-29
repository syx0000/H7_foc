# FOC 上位机调试软件 — 详细设计

## 1. 项目概述

### 1.1 目标

为 STM32H743 FOC 电机控制器（本仓库 `Core/` + `foc/`）提供一个 Python 桌面调试工具，覆盖以下场景：

- 实时观察电流环 / 速度环 / 位置环波形
- 在线修改三环 PID 参数（无需重新烧固件）
- 触发带宽测试（bwtest1/2/9）并自动绘制 Bode 图
- 触发参数辨识（Rs/Ld/Lq、ψ_f、J）+ AutoTune（电流/速度/位置 PI）
- Flash 参数读 / 写 / 擦 / 对比
- 故障码 (`ServoErrFlag`) 实时位解码 + 一键清除
- 电角度校准 (Cali) + OTA 入口（协议预留）
- 原始日志查看 / 自定义命令输入 / 日志保存

### 1.2 通信底座

复用固件已有的 USART1 文本调试协议（921600 baud, 8N1, DMA TX + DMA IDLE RX，参见 `Core/Src/usart.c`、`foc/foc_fast/foc_bsp.c::dbg_cmd_set`）。**不引入新协议**，只在上位机侧封装命令构造和响应解析。

---

## 2. 技术栈

| 组件 | 选型 | 理由 |
|------|------|------|
| 语言 | Python 3.10+ | 跨平台、生态、无需编译 |
| GUI | PySide6 (Qt6) | 商业可用、信号槽天然契合实时数据 |
| 串口 | pyserial | 标准、稳定 |
| 绘图 | pyqtgraph | C++ 后端，~30Hz 流畅滚动 |
| 数值 | numpy | 环形缓冲 / Bode 计算 |
| 测试 | pytest（轻量） | 解析器单测 |

依赖锁定见 `tools/foc_tuner/requirements.txt`：
```
PySide6>=6.6,<7.0
pyserial>=3.5
pyqtgraph>=0.13.3
numpy>=1.24
```

---

## 3. 项目结构

```
tools/foc_tuner/
├── main.py                       # 应用入口 (QApplication + MainWindow)
├── requirements.txt
├── README.md
├── FOC_Tuner.spec                # PyInstaller 打包配置
├── foc_tuner_config.json         # 用户本地 PID 参数缓存（运行时生成）
├── core/
│   ├── __init__.py
│   ├── serial_worker.py          # QThread 串口读写
│   ├── protocol.py               # 命令构造器（纯函数）
│   ├── parser.py                 # 响应行解析（注册式）
│   ├── data_model.py             # numpy 环形缓冲 + Qt 信号
│   └── units.py                  # Q10 / 角度 / 转速换算
├── gui/
│   ├── __init__.py
│   ├── main_window.py            # 主窗口（标签页容器 + 信号路由）
│   ├── serial_panel.py           # 顶栏：端口选择 + 连接 + Reset MCU
│   ├── console_widget.py         # 原始日志 + 自定义命令行 + 历史
│   ├── waveform_widget.py        # pyqtgraph 滚动波形 + 双光标
│   ├── motor_control_panel.py    # 模式 / 目标 / Run / Enable / Log 选择
│   ├── pid_panel.py              # 三环 PID + 相位补偿 + Save/Load JSON
│   ├── bandwidth_test_panel.py   # bwtest 按钮 + 结果收集
│   ├── bode_widget.py            # Bode 图（dB / phase 双子图）
│   ├── flash_panel.py            # Flash 读 / 写 / 擦 / 对比
│   ├── fault_panel.py            # ServoErrFlag 位解码表
│   └── maintenance_panel.py      # Cali + OTA 入口
└── tests/
    └── test_parser.py
```

---

## 4. 架构

### 4.1 数据流（接收方向）

```
STM32 USART1 (921600, 文本+\r\n)
     │
     ▼
SerialWorker(QThread)
  read(in_waiting) → bytearray accumulate → split('\n')
     │
     ▼ sig_line_received(str)
MainWindow._on_line_received(line)
     ├─► console.append_line(line)             # 原始日志
     ├─► parser.parse_line(line) → ParsedFrame # 结构化
     │       └─► DataModel.append(logid, ts, fields)
     │              └─► WaveformWidget (30Hz QTimer 刷新)
     ├─► bw_panel.process_line(line)           # bwtest 状态机
     ├─► flash_panel.process_line(line)        # FlashData Dump 块
     ├─► fault_panel.process_line(line)        # ServoErrFlag 解码
     ├─► pid_panel.process_line(line)          # PARAMS_BEGIN/END 块
     └─► maint_panel.process_line(line)        # Cali done / version
```

### 4.2 数据流（发送方向）

```
所有面板内部按钮点击
     │
     ▼ sig_command.emit(str)  (各面板独立信号)
MainWindow._on_command(cmd)
     │
     ▼
SerialWorker.send(cmd)        # 自动追加 \r\n（固件 DMA IDLE 必需）
     │
     ▼
STM32 USART1 RX → dbg_cmd_set()
```

### 4.3 线程模型

| 线程 | 职责 |
|------|------|
| 主线程（GUI） | Qt 事件循环、所有 widget、DataModel 增量更新、波形 30Hz 重绘 |
| SerialWorker | 串口轮询 + 行累积 + 错误恢复，仅信号跨线程传递 |

设计权衡：
- **不为 DataModel 单独开线程** — 行频率约 100 lines/s（logid 周期 10ms），主线程完全吃得下，省一次锁。
- **不为 DataModel 加锁** — `append` 和 `get_channel` 都在主线程（信号槽默认 QueuedConnection 跨线程，槽在主线程执行）。
- 波形重绘走 **QTimer 30Hz 主动拉取**，不是 `sig_new_data` 直接驱动 — 高速数据下避免槽函数被频繁调用拖慢 GUI。

### 4.4 模块依赖

```
main.py
  └─► gui.main_window
         ├─► core.serial_worker          (实例)
         ├─► core.data_model             (实例)
         ├─► core.parser                 (parse_line / set_active_logid)
         ├─► gui.serial_panel
         ├─► gui.console_widget
         ├─► gui.waveform_widget          ─► core.data_model
         ├─► gui.motor_control_panel      ─► core.protocol
         ├─► gui.pid_panel                ─► core.protocol
         ├─► gui.bandwidth_test_panel     ─► core.protocol, gui.bode_widget
         ├─► gui.flash_panel
         ├─► gui.fault_panel
         └─► gui.maintenance_panel        ─► core.protocol
```

`core/` 不依赖 `gui/`；`gui/` 之间也无横向依赖（除 `bandwidth_test_panel ↔ bode_widget` 这种内嵌组合关系）。所有跨面板通信走 MainWindow 居中转发。

---

## 5. 串口协议封装

### 5.1 命令构造器 (`core/protocol.py`)

全部为纯函数，输入参数返回字符串（不带 `\r\n`，由 `SerialWorker.send` 统一追加）。

| 函数 | 输出 | 用途 |
|------|------|------|
| `build_logid(N)` | `logid<N>` | 切换周期日志 |
| `build_logfreq(ms)` | `logfreq<N>` | 设置日志周期 |
| `build_current_pid(kp, ki, kd)` | `CurrentPIDKp<a>Ki<b>Kd<c>` | 电流环 PID |
| `build_speed_pid(...)` | `SpeedPIDKp<a>Ki<b>Kd<c>` | 速度环 PID |
| `build_position_pid(...)` | `PositionPIDKp<a>Ki<b>Kd<c>` | 位置环 PID |
| `build_runcmd(cmd, mode, target)` | `Runcmd<c>M<m>tar<t>` | 启动运行 |
| `build_enable(bool)` | `enable<0/1>` | PWM 使能 |
| `build_cali()` | `Cali` | 电角度辨识 |
| `build_version()` | `version` | 查询固件版本 |
| `build_bwtest(N)` | `bwtest<N>` | 带宽 / 辨识 / autoTune |
| `build_flash_write/erase/compare()` | `logid160/161/162` | Flash 操作 |
| `build_clear_faults()` | `logid163` | 清故障 |
| `build_reset()` | `reset` | NVIC_SystemReset |
| `build_phase_comp(...)` | 4 行: `offsetpos / offsetneg / comppos / compneg` | 相位补偿 |
| `build_save_phase_comp()` | `savephasecomp` | 相位补偿落 Flash |
| `build_query_params()` | `getparams` | 查询当前参数（PARAMS_BEGIN..END 块） |

### 5.2 响应解析器 (`core/parser.py`)

#### 注册模式

每种 logid 输出格式都是一个独立函数，用 `@register("prefix")` 装饰，新增格式零侵入：

```python
@register("current_pi:")
def _parse_current_pi(line: str) -> ParsedFrame | None:
    m = re.match(r'current_pi:\s*(-?\d+),\s*...', line)
    if not m: return None
    vals = [int(x) for x in m.groups()]
    return ParsedFrame(
        timestamp=time.perf_counter(),
        logid=40,
        fields={'I_q': vals[0]/1024.0, 'I_d': vals[1]/1024.0, ...}
    )
```

`parse_line(line)` 遍历前缀匹配；都不命中则尝试纯数字 fallback（logid 60/70/90 的"裸数字逗号分隔"格式）。

#### 当前覆盖

| logid | 前缀 | 字段（SI 单位） |
|-------|------|-----------------|
| 10 | `Angle_elec_360:` | `now_mechposition`(°), `theta_elec`, `real_position_out`(°), `real_position`(°), `dtheta_mech_rpm` |
| 30 | `current_get:` | `V_q`(V), `V_d`(V) |
| 40 | `current_pi:` | `I_q`(A), `I_d`(A), `V_q`(V), `V_d`(V), `I_q_ref`(A), `I_d_ref`(A), `I_q_ref_filt`(A) |
| 50 | (固定字段) | `vel_ref`(rpm), `vel_ref_filt`(rpm), `dtheta_mech`(rpm), `dtheta_mech_load_eq`(rpm), `vel_diff`(rpm) |
| 60 | 数字 | `CCR2/3/4` |
| 70 | 数字 | `CCR2/3/4`, `I_a/b/c`(A) |
| 90 | 数字 | `Ia_raw/Ib_raw/Ic_raw` (ADC 16-bit) |
| 100 | (固定字段) | `pos_ref`(°), `pos_out`(°), `pos_error`(°), `mech_offset`(°) |
| 110 | (ISR 计时) | `tot_us / tot_us_max / read_us / ... / cur_us_max` |

#### 单位约定（解析器统一转 SI）

固件内部：
- 位置 1°/1024 LSB → 上位机 °
- 速度 rpm × 1024（电机端） → 上位机 rpm
- 电流 Q10 A → 上位机 A
- 电压 Q10 V → 上位机 V

GUI 层不再二次转换。

#### 多行响应（状态机式）

部分输出跨多行，由对应面板自己接管：

| 块 | 起始标记 | 结束标记 | 接收方 |
|----|---------|---------|--------|
| FlashData Dump | `===== FlashData Dump` | `===== End =====` | `flash_panel.process_line` |
| Bode 数据 | 含 `Freq(Hz) Gain(dB) Phase(deg)` 表头 | 第一条非数字行 | `bandwidth_test_panel.process_line` |
| 参数查询 | `PARAMS_BEGIN` | `PARAMS_END` | `pid_panel.process_line` |
| 故障 | `FAULT! ServoErrFlag=0x...` | 单行 | `fault_panel.process_line` |
| Cali 完成 | `Cali done` | 单行 | `maintenance_panel.process_line` |

### 5.3 数据模型 (`core/data_model.py`)

- `ChannelBuffer`：固定大小 numpy 环形缓冲（默认 10000 点 ≈ 100s @ 10ms 周期）
  - `append(t, y)` O(1) 写
  - `get_data()` 返回时间序列（已展平）
- `DataModel`：channel name → ChannelBuffer 字典
  - 新通道首次出现时自动建桶（不预先约束 schema）
  - `sig_new_data(logid)` 信号供其他组件感知（实际 GUI 用 QTimer 拉取）

---

## 6. GUI 模块详细设计

### 6.1 主窗口 (`gui/main_window.py`)

**布局**：

```
┌──────────────────────────────────────────────────────────────┐
│ SerialPanel (Port / Baud / Connect / Reset MCU)              │
├──────────────────────────────────┬───────────────────────────┤
│ ┌──────────────────────────────┐ │                           │
│ │ Tabs                         │ │                           │
│ │  Motor Control | PID | BW |  │ │ ConsoleWidget             │
│ │  Flash | Faults | Maintenance│ │  (原始日志 + 输入)        │
│ └──────────────────────────────┘ │                           │
│ ┌──────────────────────────────┐ │                           │
│ │ WaveformWidget               │ │                           │
│ │  (pyqtgraph 多通道滚动)       │ │                           │
│ └──────────────────────────────┘ │                           │
└──────────────────────────────────┴───────────────────────────┘
```

**关键行为**：
- 切到 BW Test / Flash / Faults / Maintenance 时**隐藏波形区**，让结果区铺满（这些页有自己的可视化）。
- 切到 Motor Control 时把 Tab 区压缩到 220px（横向布局够用），波形最大化。
- 切到 PID Tuning 时如果已连接，自动发 `getparams` 拉一次最新参数。

**信号路由**：
所有面板的 `sig_command` 都接到 MainWindow 的统一槽，转给 `SerialWorker.send`，并在 console 镜像一行 `>>> <cmd>`。

### 6.2 串口面板 (`gui/serial_panel.py`)

- 端口枚举（`pyserial.tools.list_ports.comports()`），Refresh 按钮重扫
- 波特率下拉（默认 921600，备选 115200/57600/9600）
- Connect / Disconnect 同一按钮切换文字
- **Reset MCU** 按钮独立隔出 60px 间距，避免误点；带二次确认（`QMessageBox.question`）

### 6.3 波形 (`gui/waveform_widget.py`)

**核心选型**：pyqtgraph `PlotWidget` + 多条 `PlotDataItem`，QTimer 33ms (~30Hz) 重绘。

**功能**：
- `Pause` 暂停绘制（数据继续采集，恢复时跳跃接续）
- `Clear` 清通道数据
- `Auto Y` 自动 Y 轴
- `Follow X` + `Window (s)` 实时跟踪窗口；关掉后用户可拖拽自由缩放
- `Reset View` 一键回到当前窗口
- 通道勾选条：检测到新通道自动出现；颜色按预定义 8 色循环
- **双光标**：左键点击图区放置 A，再点放置 B，显示 t/值/Δt/频率（1/Δt）
- 时间轴显示 **相对时间**（相对于最新点），保持 0 在右侧滚动

### 6.4 电机控制 (`gui/motor_control_panel.py`)

横向两组：
- **Motor Control**：Mode 下拉（Position=1, Velocity=3, Torque=4） / Target SpinBox / Reverse 复选 / Run 按钮 / Enable PWM 切换。
  - Run 按钮按下时，固件会自动使能 PWM（TIM1 BDTR.MOE + CCER），UI 同步把 Enable 切到选中（`blockSignals` 防止重复发命令）。
- **Data Logging**：Log ID 下拉（10/30/40/50/60/70/90/100/110） / Enable 复选 / Period (ms) SpinBox。
  - Enable 关掉时发 `logid0` 停止周期日志（节省串口带宽）。

### 6.5 PID 调参 (`gui/pid_panel.py`)

四个子页面：电流环 / 速度环 / 位置环 / 相位补偿（PhaseCompTuner）。

**PIDTuner** 通用模式（每环路一份）：
- Kp / Ki / Kd 三组 Slider + DoubleSpinBox 双向绑定
- Apply to Motor 按钮：构造对应 `*PIDKp..Ki..Kd..` 命令
- Save Config / Load Config：写到 `tools/foc_tuner/foc_tuner_config.json`

**PhaseCompTuner**：
- 4 个 SpinBox：`offset_pos`, `offset_neg`（×0.1°），`comp_pos`, `comp_neg`（×0.1）
- Apply 触发 4 行 `offsetpos/offsetneg/comppos/compneg`
- Save to Flash 按钮：发 `savephasecomp`

**自动同步**：MainWindow 切到 PID 标签时调 `query_params()` → `getparams`，固件回 `PARAMS_BEGIN ... PARAMS_END` 块，`pid_panel.process_line` 解析每行 `KEY: value` 同步到 UI。

**JSON 缓存格式**（`foc_tuner_config.json`）：
```json
{
  "current_loop": {"kp": 45, "ki": 4, "kd": 0},
  "speed_loop":   {"kp": 1500, "ki": 10, "kd": 0},
  "position_loop":{"kp": 3016, "ki": 9, "kd": 0},
  "phase_comp":   {"offset_pos": 0, "offset_neg": 0, "comp_pos": 20, "comp_neg": 26}
}
```

### 6.6 带宽测试 (`gui/bandwidth_test_panel.py` + `gui/bode_widget.py`)

**按钮分组**：
- Bandwidth：bwtest1（电流环 10~2500Hz） / bwtest2（速度环 1~200Hz） / bwtest9（位置环 4~100Hz）
- Identification：bwtest3（Rs/Ld/Lq） / bwtest4（ψ_f） / bwtest5（J）
- AutoTune：bwtest6（电流 PI） / bwtest7（速度 PI） / bwtest8（位置 PI）
- One-shot：Run Full Sequence 串行下发 3→4→5→6→7→8（依赖固件命令队列）

**结果收集状态机**（`process_line`）：
1. 检测到含 `Freq(Hz)`、`Gain(dB)`、`Phase(deg)` 三个关键词的表头 → 进入收集态
2. 后续每行尝试 `split()` 取 3 个 float（freq/gain/phase）；成功就追加到列表
3. 第一条解析失败的行 → 退出收集态，把列表丢给 `BodePlotWidget.set_data`

**BodePlotWidget**：
- 上下两个 `pg.PlotWidget`：dB 子图（带 -3dB 红虚线）+ Phase 子图（带 -180° 红虚线），均 X 轴对数
- 自动计算并显示：峰值增益 / -3dB 带宽 / 0dB 穿越频率 / 相位裕度（线性插值）

### 6.7 Flash (`gui/flash_panel.py`)

三个按钮：Read（logid162） / Write（logid160） / Erase（logid161，红底警示色）。

`process_line` 用 `_in_dump_block` 标志位捕获 `===== FlashData Dump` 起到 `===== End =====` 止的整块，原样追加到 `QTextEdit`，块外只放行三个关键字（`Flash erase OK/FAIL`、`WriteDataToFlash`、`faults cleared`）。

### 6.8 故障 (`gui/fault_panel.py`)

- 静态表 10 行：bit 0..9，对应 `OverBusVolErr` / `LowBusVolErr` / `HighBoardTempErr` / `OverBusCurrentErr` / `HighMotorTempErr` / `LockedRotorErr` / `PhaseUVolErr` / `PhaseVVolErr` / `PhaseWVolErr` / `DriverChipNfault`
- `process_line` 正则匹配 `FAULT! ServoErrFlag=0x...`，解析 16 进制后逐位染色（红=触发，白=正常）
- Clear All Faults（`logid163`） / Refresh Status（`logid165`）

### 6.9 维护 (`gui/maintenance_panel.py`)

- **Cali**：要求用户先勾选"轴空载"复选才能点 Run；发 `Cali` 后等 `Cali done` 或 `Cali: Flash erase FAIL`。
- **OTA**：本地选 .bin → 计算 size + zlib.crc32 → 显示。Upload 当前 stub（弹窗提示固件 bootloader 未实现），UI 已经备好接入点。

### 6.10 控制台 (`gui/console_widget.py`)

- 上：Clear / Save Log（CSV-like 文本，带时间戳文件名）
- 中：`QPlainTextEdit`，`maxBlockCount=5000` 自动滚动丢老行，Consolas 9pt
- 下：自定义命令输入行（`_HistoryLineEdit`），Up/Down 翻 100 条历史，Enter 发送（自动 `\r\n`）；未连接时禁用

---

## 7. 关键设计决策

| # | 决策 | 理由 |
|---|------|------|
| 1 | **解析器注册模式** | `@register("prefix")` 单函数自包含。新增 logid 不动核心，单测覆盖容易。 |
| 2 | **GUI/数据同线程** | 100 lines/s 不到，省锁。SerialWorker 是唯一异步源。 |
| 3 | **波形 30Hz 拉取** | QTimer 主动 `get_channel` 比信号槽密集触发更稳。 |
| 4 | **bytearray 累积 + 4KB 溢出保护** | 防止串口卡死时无界增长；溢出保留尾部 2KB 续接。 |
| 5 | **`ascii errors='replace'`** | 偶发噪声字节不让整行报废。 |
| 6 | **解析器输出即 SI 单位** | GUI 不感知 Q10，单位转换只此一处。 |
| 7 | **命令拼装层独立纯函数** | 100% 可单测，无 Qt 依赖。 |
| 8 | **多行块状机就近放在面板** | Flash/BW/PID 各管一段，main_window 不堆积条件分支。 |
| 9 | **波形隐藏在结果型标签** | BW/Flash/Faults/Maintenance 各有专属可视化，让出垂直空间。 |
| 10 | **Reset MCU 二次确认** | 误点会丢 RAM 状态；与 Disconnect 用 60px 留白隔开。 |

---

## 8. 验证方式

### 8.1 单元测试

```bash
cd tools/foc_tuner
python tests/test_parser.py         # 或 pytest tests/
```

覆盖：
- 各 logid 行格式 → ParsedFrame 字段值正确
- 边界（负号 / 空格变体 / 不匹配返回 None）
- 单位换算（Q10 → SI）

### 8.2 集成验证

1. 接 STM32（COM4 @ 921600），点 Connect。
2. Motor Control → Log ID 选 50（速度）→ Enable 勾选 → 波形滚动起来。
3. `Runcmd2M3tar20` （Mode=Velocity, Target=20rpm 输出端） → 速度通道阶跃响应可见。
4. PID Tuning → 切到速度环 → 拖 Kp → Apply → 观察上升时间变化。
5. BW Test → bwtest2 → 等几秒 → Bode 图 + 文字结果同步刷新。
6. Flash → Read → `===== FlashData Dump ... ===== End =====` 整块完整捕获。
7. 故意触发欠压 → Faults 表 bit1 染红 → Clear → 染色复原。
8. 拔串口 / 重插 → 不崩溃；Reconnect 后波形继续。

### 8.3 已知约束

- 本机 `python` 是 Windows App 别名（静默退出），需用完整路径或 PowerShell 启动。
- bwtest 全序列（3→4→5→6→7→8）当前是简单串行下发，依赖固件命令队列，不主动等每一步完成。
- OTA 上传协议 stub，等固件 bootloader 接收端实现后再补。

---

## 9. 扩展指引

### 9.1 新增一种 logid 输出

1. 固件侧 printf 行格式定下来，比如 `mit_state: %d,%d,%d`。
2. `core/parser.py` 加：
   ```python
   @register("mit_state:")
   def _parse_mit_state(line):
       m = re.match(r'mit_state:\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)', line)
       if not m: return None
       v = [int(x) for x in m.groups()]
       return ParsedFrame(time.perf_counter(), 120, {
           'mit_pos': v[0]/1024.0, 'mit_vel': v[1]/1024.0, 'mit_torque': v[2]/1024.0
       })
   ```
3. 在 `get_channel_names` 字典里加 `120: ['mit_pos', 'mit_vel', 'mit_torque']`。
4. `motor_control_panel.py` 的 logid 下拉加一项 `120 - MIT State`。
5. 写 `test_parser.py` 用例。完成。

### 9.2 新增一个调试面板

1. 在 `gui/` 加文件，类继承 `QWidget`，暴露 `sig_command = Signal(str)`，可选 `process_line(self, line)`。
2. `main_window.py` 实例化，加进 `left_tabs.addTab(...)`。
3. 在 `_on_command` 里把 `sig_command` 接到 `_serial.send`；如有 `process_line`，加进 `_on_line_received` 的转发链。
4. 如果不需要波形，把 tab 索引加进 `WAVEFORM_HIDDEN_TABS`。

---

## 10. OTA 固件升级详细设计

### 10.1 硬件约束

| 项 | 值 | 来源 |
|----|----|----|
| Flash 总容量 | 2MB | RM0433 §4，2 Bank × 1MB |
| Bank1 范围 | `0x08000000 ~ 0x080FFFFF` (1MB) | scatter `LR_IROM1` |
| Bank2 范围 | `0x08100000 ~ 0x081FFFFF` (1MB) | RM0433 |
| 扇区大小 | 128KB（每 Bank 8 个） | H743 固定 |
| 写粒度 | 32 字节（256 位 FLASHWORD） | `flash_port.h` `FLASH_WRITE_GRANULARITY` |
| 当前应用大小 | Code 105KB + RO 13.8KB ≈ **120KB** | `MDK-ARM` 编译规模 |
| 参数区 | Bank2 Sector 7 (`0x081E0000`, 128KB) | `flash_port.h` |
| **可用 OTA 空间** | Bank2 Sector 0~6 (`0x08100000 ~ 0x081DFFFF`, **896KB**) | 推导 |

应用大小 120KB 远小于 1MB，**双区 A/B 槽**方案完全可行。

### 10.2 Flash 布局（双槽 A/B）

```
0x08000000 ┌────────────────────────────┐
           │ Bootloader (Sector 0)      │  128KB  ← 新增
0x08020000 ├────────────────────────────┤
           │ App-A (Sector 1~6)         │  768KB  ← 主应用槽，正常运行
0x080E0000 ├────────────────────────────┤
           │ App-A 元数据 (Sector 7)    │  128KB  ← header (size/crc/version)
0x08100000 ├════════════════════════════┤  ← Bank2 起始
           │ App-B (Sector 0~6)         │  768KB  ← OTA 接收槽
0x081E0000 ├────────────────────────────┤
           │ FOC 参数 (现有 Sector 7)   │  128KB  ← 已有，不动
0x08200000 └────────────────────────────┘
```

**关键决策**：
- **Bootloader 独立 Sector 0**（128KB，远超实际所需的 ~20KB），擦写应用时绝不擦到自己。
- App-A 占 Bank1 后 768KB，App-B 占 Bank2 前 768KB，两槽对称。
- Bank1/Bank2 物理独立，**Bank2 写入时 Bank1 仍可执行**（H7 双 Bank 特性），是写 Flash 不卡 FOC 的硬件基础。
- 元数据 (header) 单独占一个扇区，避免应用更新时元数据被同步擦掉造成"写一半掉电变砖"。

### 10.3 双槽 Header 格式

每个 App 槽的元数据扇区前 128 字节：

```c
typedef struct {
    uint32_t magic;        // 'FOCA' = 0x41434F46，标识有效
    uint32_t version;      // 固件版本号 (主.次.补丁 编码)
    uint32_t app_size;     // 应用字节数（不含 header）
    uint32_t app_crc32;    // 应用区 CRC32（同 flash_port.c::Flash_Crc32）
    uint32_t boot_count;   // 已启动次数（首次 0，bootloader 每次 +1，应用启动成功后清 0）
    uint32_t flags;        // bit0=valid, bit1=tested, bit2=rollback_pending
    uint32_t build_time;   // Unix timestamp
    uint32_t reserved[25]; // 凑齐 128B
} app_header_t;
```

CRC32 复用现有 `Flash_Crc32`（IEEE 802.3 标准，poly 0xEDB88320），上位机 `zlib.crc32` 输出与之**比特等价**，无需另写。

### 10.4 启动流程（Bootloader）

```
上电 / 复位
  │
  ▼
读 App-A header + App-B header
  │
  ├─► 都无效（magic 不对）→ 卡死，错误指示灯
  │
  ▼
有效槽筛选：magic==FOCA && CRC 校验通过 && boot_count < 3
  │
  ├─► 两槽都有效 → 选 version 较高的
  ├─► 仅一槽有效 → 选它
  │
  ▼
boot_count += 1 写回 header
  │
  ▼
跳转到 app 入口（Set MSP + 跳 Reset_Handler）
```

应用启动成功（如完成 `Init_foc` + 闭环跑 3 秒）后，回头清 `boot_count = 0`。

**回滚机制**：连续 3 次 boot 失败（boot_count 累计到 3 还没被应用清零），下次启动时 bootloader 标记该槽 `flags |= rollback_pending`，切到另一槽。

### 10.5 串口传输协议

#### 5.1 帧格式

**沿用现有文本命令风格的关键帧 + 二进制 chunk**，方便复用 `dbg_cmd_set` 的解析框架：

| 阶段 | 方向 | 帧格式 | 说明 |
|------|------|--------|------|
| 协商 | 上→下 | `otabegin SIZE=<n> CRC=0x<hex> VER=<v>\r\n` | 文本，固件擦目标槽并回 ACK |
| ACK | 下→上 | `OTA_READY chunk=1024\r\n` 或 `OTA_ERR <reason>\r\n` | 文本 |
| 数据 | 上→下 | `OTA_DATA <seq:u16> <len:u16> <payload> <crc16:u16>` | 二进制（先发 4 字节文本头 `OD\xff\xff` 标识帧起始，然后是定长二进制） |
| 块 ACK | 下→上 | `OTA_ACK <seq>\r\n` 或 `OTA_NAK <seq> <reason>\r\n` | 文本 |
| 结束 | 上→下 | `otaend\r\n` | 触发整片 CRC 校验 + 写 header |
| 结束 ACK | 下→上 | `OTA_DONE\r\n` 或 `OTA_FAIL <reason>\r\n` | 文本 |
| 切换 | 上→下 | `otaswap\r\n` | 主动重启进 bootloader 切槽 |

#### 5.2 块大小选择

- chunk = 1024B（1KB）→ 768KB 固件 = **768 块**
- 921600 baud 理论上行 ~92KB/s，加 ACK 往返开销 → **预估 ~12s** 烧 768KB
- 太小（如 64B）会被 ACK 往返时延吃掉吞吐；太大（如 8KB）超过 STM32 USART DMA RX buffer，需要拆。

#### 5.3 错误处理

- 单块 NAK：上位机重传该 seq，最多 3 次
- 连续 5 块 NAK：上位机放弃，发 `otaabort`，固件清掉接收槽 header（写全 0xFF）
- 5 秒无 ACK：上位机超时，同上

### 10.6 上位机实现（`tools/foc_tuner/`）

#### 6.1 协议层 (`core/protocol.py`)

新增构造器：
```python
def build_ota_begin(size: int, crc32: int, version: str) -> str:
    return f"otabegin SIZE={size} CRC=0x{crc32:08X} VER={version}"

def build_ota_end() -> str: return "otaend"
def build_ota_abort() -> str: return "otaabort"
def build_ota_swap() -> str: return "otaswap"

def build_ota_data_frame(seq: int, payload: bytes) -> bytes:
    """二进制 chunk 帧：'OD'(2) + seq(2 LE) + len(2 LE) + payload + crc16(2 LE)"""
    import struct
    header = b'OD' + struct.pack('<HH', seq, len(payload))
    crc = _crc16_modbus(header + payload)
    return header + payload + struct.pack('<H', crc)
```

#### 6.2 上传 Worker (`core/ota_worker.py`，新增)

`QThread` 子类，独立线程跑发送循环（避免阻塞 GUI）：

```python
class OTAUploader(QThread):
    sig_progress = Signal(int, int)        # bytes_sent, total
    sig_status   = Signal(str)             # 状态文本
    sig_done     = Signal(bool, str)       # success, message

    def __init__(self, serial_worker, bin_data, version):
        super().__init__()
        self._sw = serial_worker
        self._bin = bin_data
        self._version = version
        self._ack_queue = queue.Queue()    # 接收线程把 ACK 塞进来

    def on_line(self, line: str):
        if line.startswith(('OTA_READY', 'OTA_ACK', 'OTA_NAK',
                            'OTA_DONE', 'OTA_FAIL', 'OTA_ERR')):
            self._ack_queue.put(line)

    def run(self):
        # 1. 发 otabegin，等 OTA_READY
        # 2. 循环 chunk：发 OTA_DATA 帧 → 等 OTA_ACK <seq> 或 NAK 重传
        # 3. 发 otaend，等 OTA_DONE
        # 4. emit sig_done
```

`SerialWorker.sig_line_received` 接到 `OTAUploader.on_line`，让上传线程感知 ACK；同时 `serial_worker.send` 已经线程安全。

#### 6.3 二进制发送

发送 chunk 时直接走 `serial_worker._port.write(bytes)`，**绕过 `send()` 的 ASCII 转换**。需要给 `SerialWorker` 加一个 `send_bytes(data: bytes)` 方法（同样 mutex 保护），不影响现有文本通道。

#### 6.4 GUI (`gui/maintenance_panel.py` 改造）

现有面板已经有：文件选择、size、crc32、Upload 按钮、进度条。要做的：

1. Browse 时除了算 crc32，也支持从 .axf/.bin 同名 .ver 文件读版本号（可选，缺失就让用户在 LineEdit 里手填）。
2. Upload 按钮接 `OTAUploader`，把进度信号连到 `QProgressBar`，状态信号连到 log。
3. 完成后弹 "Upload OK, swap & reboot now?" 确认框，点 Yes 发 `otaswap`。
4. 进行中禁用其他面板（防止 PID 改动撞 OTA），完成或失败恢复。

### 10.7 固件侧实现要点（不在本文件实施，仅列设计接口）

#### 7.1 应用侧（本仓库 `Core/Src/usart.c` + 新增 `ota_app.c`）

- `dbg_cmd_set` 增加 `otabegin` / `otaend` / `otaabort` / `otaswap` 命令解析。
- 新增 OTA 状态机模块 `Core/Src/ota_app.c`：
  - `ota_begin(size, crc, ver)` → 擦 App-B（6 个扇区，Bank2 Sector 0~5），耗时 ~1.5s（H7 单扇区擦 ~250ms），擦完回 `OTA_READY`。
  - `ota_data(seq, payload)` → 累积到 32B 边界后调 `Flash_WriteData` 直写 Bank2，每块写完回 `OTA_ACK <seq>`。
  - `ota_end()` → 把元数据扇区 (Bank2 Sector 6) 写入 `app_header_t`（magic/size/crc/version/flags=valid），整片读回算 CRC 校验，成功回 `OTA_DONE`。
  - `ota_swap()` → `NVIC_SystemReset()`，bootloader 看到 App-B 比 App-A 新就跳 B。
- **关键**：擦/写 Flash 期间 **关 ADC 中断 (FOC ISR)**，因为 H7 写 Flash 有几十毫秒的 stall（同 Bank 才 stall，跨 Bank 写不影响执行 — 这正是把应用放 Bank1、OTA 槽放 Bank2 的核心收益）。但保险起见，进 OTA 时先发 `enable0` 关 PWM，结束再让用户决定。

#### 7.2 Bootloader（新增独立工程）

- 独立 `MDK-ARM` 子工程，scatter 锁定 `0x08000000 ~ 0x0801FFFF`（128KB）。
- 仅依赖 HAL 的 Flash + UART + SysTick，体积应该 < 30KB。
- 链接产物 `bootloader.bin`，烧到 Bank1 Sector 0；之后所有应用更新都走 OTA。
- **首次部署**：用 ST-Link / DAP 烧一次 bootloader + App-A，之后脱机 OTA。

### 10.8 安全 / 鲁棒性

| 风险 | 对策 |
|------|------|
| 写 App-B 中途掉电 | App-B header 最后写，magic 不对 bootloader 不会跳；下次重新 OTA。 |
| 新固件能启动但跑飞 | boot_count 计数 + 应用启动 3 秒后清零；连续 3 次没清就回滚 App-A。 |
| Bootloader 自己挂了 | Bootloader 区域用 STM32 选项字节 **写保护**（OPTSR.WRP），FOC 应用通过 OTA 永远碰不到 Bank1 Sector 0。 |
| 升级到不兼容版本（参数区结构变了） | header 里加 `param_struct_ver` 字段，不一致时新固件首次启动重新跑辨识写参数区（现有逻辑已有 `FLASH_STRUCT_VERSION` 机制）。 |
| 串口噪声导致 chunk 损坏 | chunk 内 CRC16-MODBUS + 整片 CRC32 双重校验。 |
| 上位机崩溃中途断开 | 固件 5 秒无数据自动 timeout，clear App-B header；下次重 OTA 即可。 |

### 10.9 测试方案

| 阶段 | 验证项 |
|------|--------|
| Unit | `OTAUploader` 用 stub serial 跑完整序列；CRC32 / CRC16 跨语言一致性。 |
| 集成 | 真硬件烧 1KB / 100KB / 768KB 三档；故意中断 chunk 序列验证重传；故意改一个字节验证整片 CRC 失败。 |
| 故障注入 | OTA 进行中按 Reset → bootloader 跳 App-A 回正常运行；连发 3 个坏固件验证回滚。 |
| 性能 | 921600 baud 下 768KB 实测耗时（目标 < 15s）。 |

### 10.10 实施顺序

1. **第一阶段（应用侧 OTA 接收器）**：直接在现有应用里加 `ota_begin/data/end` 命令处理，写 Bank2 Sector 0~5 当备份槽（不切槽，仅打通"上位机能可靠传 768KB 到 Flash"这个环节）。验收：上位机传完后用 `Flash_ReadData` 读回比对 CRC32 OK。
2. **第二阶段（Bootloader 工程）**：独立 MDK 子工程，实现选槽 + 跳转 + 回滚。和应用之间通过 header 通信。
3. **第三阶段（联调）**：bootloader + 应用 + 上位机三方跑通整条 OTA → swap → 启动新固件。

当前 `gui/maintenance_panel.py` 的 OTA stub UI 直接对应第一阶段的可视面板，协议落地后只要把 stub 的 `_on_upload_clicked` 替换成 `OTAUploader` 实例化即可。

---

## 11. 阶段进度（实际状态）

| 阶段 | 范围 | 状态 |
|------|------|------|
| Phase 1 | 串口 / 波形 / 基本控制 / 日志 | ✅ 已完成 |
| Phase 2 | 三环 PID 在线调参 + JSON 持久化 + 相位补偿 | ✅ 已完成 |
| Phase 3 | 带宽测试 + Bode 图 + 一键辨识序列 | ✅ 已完成 |
| Phase 4 | Flash 管理 + 故障诊断 + Cali + OTA 入口 | ✅ 已完成（OTA 协议 stub） |
| 后续 | 历史 CSV 导出 / 波形截图 / 二进制高速协议 / OTA 协议落地 | ⏸ 待办 |
