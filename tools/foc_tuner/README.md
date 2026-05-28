# FOC Motor Tuner

基于 STM32H743 FOC 电机控制器的 Python 桌面调试工具。

## 功能特性

### ✅ 已完成功能

**Phase 1: 基础功能**
- ✅ 串口连接管理（COM 端口选择，921600 baud）
- ✅ 实时波形显示（pyqtgraph 滚动曲线，30Hz 刷新）
- ✅ 原始日志查看器
- ✅ 电机基本控制（Run/Stop, 模式选择，目标设定）
- ✅ logid 切换（10/30/40/50/60/70/90/100/110）
- ✅ 文本协议解析器（注册式，易扩展）

**Phase 2: PID 在线调参**
- ✅ 三环 PID 滑块调参（电流/速度/位置）
- ✅ 参数本地 JSON 保存/加载
- ✅ 实时下发到电机

**Phase 3: 带宽测试 + Bode 图**
- ✅ 带宽测试按钮（bwtest1-9）
- ✅ Bode 图显示（幅频/相频曲线）
- ✅ 自动解析测试结果
- ✅ 一键辨识流程（Rs/Ld/Lq → ψ_f → J → autoTune）

**Phase 4: Flash 管理 + 故障诊断**
- ✅ Flash 参数读取/写入/擦除
- ✅ RAM vs Flash 参数对比
- ✅ 故障码实时解析（ServoErrFlag 位解码）
- ✅ 故障清除功能

## 安装

### 依赖

需要 Python 3.10+ 和以下包：

```bash
pip install PySide6 pyserial pyqtgraph numpy
```

或使用 requirements.txt：

```bash
pip install -r requirements.txt
```

### 运行

```bash
cd tools/foc_tuner
python main.py
```

## 使用说明

### 连接硬件

1. 选择 COM 端口（默认 COM4）
2. 波特率保持 921600
3. 点击 "Connect"

### 查看波形

1. 在 "Motor Control" 标签页的 "Data Logging" 区域选择 logid：
   - **logid 40** - 电流环（I_q, I_d, V_q, V_d, I_q_ref 等）
   - **logid 50** - 速度环（vel_ref, dtheta_mech 等）
   - **logid 100** - 位置环（pos_ref, pos_out, pos_error）
2. 调整 Period (ms) 控制刷新率（默认 10ms）
3. 波形自动滚动，点击 "Pause" 暂停（数据继续采集）
4. "Auto Y" 勾选时自动缩放 Y 轴

### 调参 PID

1. 切换到 "PID Tuning" 标签页
2. 选择要调的环路（Current / Speed / Position）
3. 拖动滑块或输入数值调整 Kp/Ki/Kd
4. 点击 "Apply to Motor" 下发参数
5. 观察波形响应，迭代调整
6. 满意后点击 "Save Config" 保存到本地 JSON

### 带宽测试

1. 切换到 "BW Test" 标签页
2. 点击对应按钮：
   - **Current Loop (bwtest1)** - 电流环带宽测试（10-2500Hz）
   - **Speed Loop (bwtest2)** - 速度环带宽测试（1-200Hz）
   - **Position Loop (bwtest9)** - 位置环带宽测试（4-100Hz）
3. 等待测试完成（几秒到几十秒）
4. Bode 图自动显示幅频/相频曲线
5. 右侧文本框显示详细数据

### 参数辨识 + AutoTune

1. 在 "BW Test" 标签页点击：
   - **Rs/Ld/Lq (bwtest3)** - 电阻/电感辨识
   - **Flux ψ_f (bwtest4)** - 磁链辨识
   - **Inertia J (bwtest5)** - 惯量辨识
2. 辨识完成后自动写入 Flash
3. 点击 AutoTune 按钮：
   - **Current PI (bwtest6)** - 电流环自动整定
   - **Speed PI (bwtest7)** - 速度环自动整定
   - **Position PI (bwtest8)** - 位置环自动整定
4. 或点击 "Run Full Sequence" 一键完成全流程

### Flash 管理

1. 切换到 "Flash" 标签页
2. **Read Flash (logid162)** - 读取并对比 RAM vs Flash 参数
3. **Write to Flash (logid160)** - 保存当前 RAM 参数到 Flash
4. **Erase Flash (logid161)** - 擦除 Flash（下次启动重新初始化）

### 故障诊断

1. 切换到 "Faults" 标签页
2. 故障发生时，表格自动高亮显示故障位
3. 查看故障描述（母线过压/欠压/过流/堵转/缺相等）
4. 排除故障原因后，点击 "Clear All Faults" 清除
5. 点击 "Refresh Status" 查询当前故障状态

## 架构说明

### 目录结构

```
tools/foc_tuner/
├── main.py                      # 入口
├── requirements.txt
├── core/                        # 核心逻辑（非 GUI）
│   ├── serial_worker.py         # QThread 串口 I/O
│   ├── protocol.py              # 命令构造器
│   ├── parser.py                # 响应解析器（注册式）
│   ├── data_model.py            # numpy 环形缓冲
│   └── units.py                 # 单位转换
├── gui/                         # PySide6 界面
│   ├── main_window.py           # 主窗口
│   ├── serial_panel.py          # 串口连接面板
│   ├── waveform_widget.py       # 波形显示
│   ├── motor_control_panel.py   # 电机控制
│   ├── pid_panel.py             # PID 调参（Phase 2）
│   ├── bode_widget.py           # Bode 图（Phase 3）
│   ├── bandwidth_test_panel.py  # 带宽测试（Phase 3）
│   ├── flash_panel.py           # Flash 管理（Phase 4）
│   ├── fault_panel.py           # 故障诊断（Phase 4）
│   └── console_widget.py        # 日志查看器
└── tests/
    └── test_parser.py           # 解析器单元测试
```

### 数据流

```
STM32 USART1 → SerialWorker(QThread) → sig_line_received
  → MainWindow._on_line_received → console + parser.parse_line()
  → DataModel.append() → WaveformWidget (30Hz QTimer 刷新)
```

### 线程模型

- **主线程**：GUI 事件循环，DataModel，波形刷新
- **SerialWorker 线程**：串口读写，行累积，信号发射

### 扩展解析器

添加新 logid 格式只需在 `core/parser.py` 中添加一个函数：

```python
@register("your_prefix:")
def _parse_your_logid(line: str) -> ParsedFrame | None:
    # 正则匹配 + 返回 ParsedFrame
    ...
```

## 串口协议

### 发送命令（文本，\r\n 结尾）

- `logid<N>` - 选择周期日志输出
- `logfreq<N>` - 设置日志周期（ms）
- `Runcmd<cmd>M<mode>tar<value>` - 电机控制
- `enable<0/1>` - PWM 使能/失能
- `CurrentPIDKp<a>Ki<b>Kd<c>` - 设置电流环 PID
- `SpeedPIDKp<a>Ki<b>Kd<c>` - 设置速度环 PID
- `PositionPIDKp<a>Ki<b>Kd<c>` - 设置位置环 PID
- `bwtest<N>` - 带宽测试/辨识
- `Cali` - 电角度校准
- `version` - 查询固件版本

### 接收响应（printf 文本，\r\n 结尾）

详见 `core/parser.py` 中的注册函数。

## 单位约定

固件内部单位：
- 位置：1°/1024 LSB（输出端）
- 速度：rpm × 1024（电机端），输出端 = 电机端/25
- 电流：Q10 A（1024 = 1A）
- 电压：Q10 V（1024 = 1V）

解析器输出：SI 单位（A, V, °, rpm），GUI 层无需再转换。

## 测试

```bash
cd tools/foc_tuner
python tests/test_parser.py
```

## 已知问题

- 本机 `python` 命令是 Windows App 别名（不可用），需用完整路径或 PowerShell
- 全序列测试（bwtest3-8）需要等待每个测试完成，当前实现是简单串行发送（固件会排队执行）

## 未来改进

- [ ] 历史数据 CSV 导出
- [ ] 波形截图保存
- [ ] 参数配置模板管理
- [ ] 二进制高速协议支持（替代文本协议，提升采样率）

## 开发路线

- [x] Phase 1: 串口 + 波形 + 基本控制
- [x] Phase 2: PID 在线调参 + 参数保存
- [x] Phase 3: 带宽测试 + Bode 图
- [x] Phase 4: Flash 管理 + 故障诊断

## License

MIT
