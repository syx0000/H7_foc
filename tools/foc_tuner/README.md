# FOC Motor Tuner

基于 STM32H743 FOC 电机控制器的 Python 桌面调试工具，支持**串口**和 **CAN-FD** 双通道。

## 功能特性

### ✅ 已完成功能

**Phase 1: 基础功能**
- ✅ **双通道支持**：串口（USART1, 921600 baud）+ CAN-FD（创芯 USBCANFD-2CH）
- ✅ 连接配置自动保存（下次启动自动选择上次的后端）
- ✅ 实时波形显示（pyqtgraph 滚动曲线，30Hz 刷新）
- ✅ 原始日志查看器
- ✅ 电机基本控制（Run/Stop, 模式选择，目标设定）
- ✅ logid 切换（10/30/40/50/60/70/90/100/110）
- ✅ 文本协议解析器（注册式，易扩展）

**Phase 2: PID 在线调参**
- ✅ 三环 PID 滑块调参（电流/速度/位置）
- ✅ 相位补偿调参（正反转 offset + 速度相关补偿）
- ✅ 参数本地 JSON 保存/加载
- ✅ 参数查询功能（getparams）
- ✅ 实时下发到电机

**Phase 3: 带宽测试 + Bode 图**
- ✅ 带宽测试按钮（bwtest1-9）
- ✅ Bode 图显示（幅频/相频曲线）
- ✅ 自动解析测试结果
- ✅ 一键辨识流程（Rs/Ld/Lq → ψ_f → J → autoTune）
- ✅ 电角度标定（Cali）

**Phase 4: Flash 管理 + 故障诊断**
- ✅ Flash 参数读取/写入/擦除
- ✅ RAM vs Flash 参数对比（串口详细输出 / CAN 摘要输出）
- ✅ 故障码实时解析（ServoErrFlag 位解码）
- ✅ 故障清除功能

**Phase 5: CAN-FD 调试协议**
- ✅ 完整 CAN 调试协议（0x7E0~0x7E6）
- ✅ 所有命令文本反馈（Flash/PID/Enable/Cali/bwtest）
- ✅ 二进制日志解析 + 文本消息支持
- ✅ 万里扬控制协议（速度/位置/转矩模式）

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

### 打包 exe（可选）

生成单文件可执行程序（无需 Python 环境）：

```bash
cd tools/foc_tuner
python -m PyInstaller FOC_Tuner.spec --noconfirm
```

生成的 `dist/FOC_Tuner.exe` 可直接分发给其他用户。

## 使用说明

### 连接硬件

**串口模式：**
1. 选择 COM 端口（默认 COM4）
2. 波特率保持 921600
3. 点击 "Connect"

**CAN-FD 模式：**
1. 连接创芯 USBCANFD-2CH 硬件
2. 选择 Backend: "CAN-FD"
3. 设置通道（CH: 0 或 1）
4. 仲裁位速率默认 1Mbps，数据位速率默认 5Mbps
5. 点击 "Connect"

### 查看波形

1. 在 "Motor Control" 标签页的 "Data Logging" 区域：
   - 勾选 "Enable" 复选框开启日志
   - 选择 logid：
     - **logid 40** - 电流环（I_q, I_d, V_q, V_d, I_q_ref 等）
     - **logid 50** - 速度环（vel_ref, dtheta_mech 等）
     - **logid 100** - 位置环（pos_ref, pos_out, pos_error）
   - 调整 Period (ms) 控制刷新率（默认 10ms）
2. 波形自动滚动，点击 "Pause" 暂停（数据继续采集）
3. "Auto Y" 勾选时自动缩放 Y 轴

### 调参 PID

1. 切换到 "PID Tuning" 标签页
2. 点击 "Read from MCU" 查询当前参数
3. 选择要调的环路（Current / Speed / Position）
4. 拖动滑块或输入数值调整 Kp/Ki/Kd
5. 点击 "Apply to Motor" 下发参数
6. 观察波形响应，迭代调整
7. 满意后点击 "Save Config" 保存到本地 JSON

### 相位补偿调参

1. 在 "PID Tuning" 标签页切换到 "Phase Comp" 标签
2. 调整 4 个参数：
   - **Offset Pos** - 正转固定偏置（×0.1°）
   - **Offset Neg** - 反转固定偏置（×0.1°）
   - **Comp Pos** - 正转速度相关补偿（×0.1）
   - **Comp Neg** - 反转速度相关补偿（×0.1）
3. 点击 "Apply to Motor" 应用
4. 点击 "Save to Flash" 保存到 Flash

### 带宽测试

1. 切换到 "BW Test" 标签页
2. 点击对应按钮：
   - **Current Loop (bwtest1)** - 电流环带宽测试（10-2500Hz）
   - **Speed Loop (bwtest2)** - 速度环带宽测试（1-200Hz）
   - **Position Loop (bwtest9)** - 位置环带宽测试（4-100Hz）
3. 等待测试完成（控制台显示 "bwtest{N} done"）
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
2. **Read Flash (logid162)** - 对比 RAM vs Flash
   - 串口模式：详细 30+ 行对比输出
   - CAN 模式：仅输出差异摘要
3. **Write to Flash (logid160)** - 保存当前 RAM 参数到 Flash
4. **Erase Flash (logid161)** - 擦除 Flash（下次启动重新初始化）

### 故障诊断

1. 切换到 "Faults" 标签页
2. 故障发生时，表格自动高亮显示故障位
3. 查看故障描述（母线过压/欠压/过流/堵转/缺相等）
4. 排除故障原因后，点击 "Clear All Faults" 清除
5. 点击 "Refresh Status" 查询当前故障状态

### 电角度标定

1. 切换到 "Maintenance" 标签页
2. 点击 "Run Calibration" 按钮
3. 等待标定完成（控制台显示 "Cali done"）
4. 结果自动保存到 Flash

## 架构说明

### 目录结构

```
tools/foc_tuner/
├── main.py                      # 入口
├── requirements.txt
├── FOC_Tuner.spec               # PyInstaller 打包配置
├── core/                        # 核心逻辑（非 GUI）
│   ├── serial_worker.py         # QThread 串口 I/O
│   ├── can_worker.py            # QThread CAN-FD I/O（镜像 SerialWorker 接口）
│   ├── protocol.py              # 命令构造器（文本协议）
│   ├── parser.py                # 响应解析器（注册式）
│   ├── data_model.py            # numpy 环形缓冲
│   └── units.py                 # 单位转换
├── gui/                         # PySide6 界面
│   ├── main_window.py           # 主窗口
│   ├── serial_panel.py          # 串口/CAN 连接面板
│   ├── waveform_widget.py       # 波形显示
│   ├── motor_control_panel.py   # 电机控制
│   ├── pid_panel.py             # PID 调参
│   ├── bode_widget.py           # Bode 图
│   ├── bandwidth_test_panel.py  # 带宽测试
│   ├── flash_panel.py           # Flash 管理
│   ├── fault_panel.py           # 故障诊断
│   ├── maintenance_panel.py     # 维护工具（Cali, OTA, 版本查询）
│   └── console_widget.py        # 日志查看器
└── tests/
    └── test_parser.py           # 解析器单元测试
```

### 数据流

**串口模式：**
```
STM32 USART1 → SerialWorker(QThread) → sig_line_received
  → MainWindow._on_line_received → console + parser.parse_line()
  → DataModel.append() → WaveformWidget (30Hz QTimer 刷新)
```

**CAN 模式：**
```
STM32 FDCAN → CXCanFD → CanWorker(QThread) → 二进制帧 → 文本转换
  → sig_line_received → (与串口相同的处理流程)
```

### 线程模型

- **主线程**：GUI 事件循环，DataModel，波形刷新
- **SerialWorker / CanWorker 线程**：I/O 读写，行累积，信号发射

### 扩展解析器

添加新 logid 格式只需在 `core/parser.py` 中添加一个函数：

```python
@register("your_prefix:")
def _parse_your_logid(line: str) -> ParsedFrame | None:
    # 正则匹配 + 返回 ParsedFrame
    ...
```

## 协议说明

### 串口协议（文本，\r\n 结尾）

**发送命令：**
- `logid<N>` - 选择周期日志输出
- `logfreq<N>` - 设置日志周期（ms）
- `Runcmd<cmd>M<mode>tar<value>` - 电机控制
- `enable<0/1>` - PWM 使能/失能
- `CurrentPIDKp<a>Ki<b>Kd<c>` - 设置电流环 PID
- `SpeedPIDKp<a>Ki<b>Kd<c>` - 设置速度环 PID
- `PositionPIDKp<a>Ki<b>Kd<c>` - 设置位置环 PID
- `offsetpos<n>` / `offsetneg<n>` / `comppos<n>` / `compneg<n>` - 相位补偿
- `bwtest<N>` - 带宽测试/辨识
- `Cali` - 电角度校准
- `version` - 查询固件版本
- `getparams` - 查询所有 PID + 相位补偿参数

**接收响应：**

详见 `core/parser.py` 中的注册函数。

### CAN-FD 调试协议（二进制）

详见 `tools/canfd/CAN_DEBUG_DESIGN.md`

**CAN ID 分配：**
- `0x7E0` - CMD（PC → MCU 命令）
- `0x7E1` - RESP（MCU → PC 响应）
- `0x7E2` - LOG（MCU → PC 周期日志，二进制）
- `0x7E3` - EVENT（MCU → PC 事件通知）
- `0x7E6` - TEXT（MCU → PC 文本消息）

**命令列表：**
- `0x00` - PING
- `0x01` - VERSION
- `0x02` - RESET
- `0x03` - GET_PARAMS
- `0x10` - LOGID_SET
- `0x11` - LOGFREQ_SET
- `0x20/0x21/0x22` - CUR/SPD/POS PID_SET
- `0x40` - FLASH_WRITE
- `0x41` - FLASH_ERASE
- `0x42` - FLASH_COMPARE
- `0x43` - FAULT_CLR
- `0x50` - ENABLE
- `0x52` - PHASE_COMP_SET
- `0x53` - PHASE_COMP_SAVE
- `0x5F` - CALI
- `0x60` - BWTEST

**文本反馈：**

所有操作都会通过 0x7E6 发送文本确认：
- PID 设置：`Current PID: 45/4/0`
- Flash 操作：`Flash write OK` / `Flash erase OK`
- 使能：`PWM enabled` / `PWM disabled`
- 标定：`Cali done`
- 测试：`bwtest3 done`
- 相位补偿：`PhaseComp: 0/0/23/26`

### 万里扬控制协议

**速度模式（0x200）：**
- 归一化映射：±20 rad/s → uint16
- 3 字节：[v_low, v_high, node_id]

**位置模式（0x400）：**
- 归一化映射：±7 rad → uint24
- 6 字节：[p_low, p_mid, p_high, v_low, v_high, node_id]

**转矩模式（0x300）：**
- 归一化映射：±500 N·m → uint16
- 3 字节：[t_low, t_high, node_id]

**使能/失能（0x701）：**
- 2 字节：[node_id, enable]

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

## CAN vs 串口功能对比

| 功能 | 串口 | CAN-FD | 备注 |
|------|------|--------|------|
| 电机控制 | ✅ | ✅ | 万里扬协议 |
| PID 调参 | ✅ | ✅ | 完全支持 |
| 参数查询 | ✅ | ✅ | getparams |
| 相位补偿 | ✅ | ✅ | 4 合 1 自动合并 |
| Flash 管理 | ✅ | ✅ | 写入/擦除/对比 |
| 电角度标定 | ✅ | ✅ | Cali |
| 带宽测试 | ✅ | ✅ | bwtest 1-10 |
| 实时日志 | ✅ | ✅ | 二进制高效传输 |
| 文本反馈 | ✅ | ✅ | 0x7E6 TEXT 帧 |
| Flash 详细对比 | ✅ | ⚠️ | CAN 仅摘要，详细走串口 |
| OTA 固件升级 | ✅ | ❌ | 大数据传输建议串口 |

## 已知问题

- Windows 环境 `python` 命令可能是 App 别名，需用完整路径或 PowerShell
- CAN-FD 需要创芯硬件（USBCANFD-2CH）和驱动 DLL

## 未来改进

- [ ] 历史数据 CSV 导出
- [ ] 波形截图保存
- [ ] 参数配置模板管理
- [ ] CAN-FD OTA 支持（分片传输）

## 开发路线

- [x] Phase 1: 串口 + 波形 + 基本控制
- [x] Phase 2: PID 在线调参 + 参数保存
- [x] Phase 3: 带宽测试 + Bode 图
- [x] Phase 4: Flash 管理 + 故障诊断
- [x] Phase 5: CAN-FD 调试协议 + 文本反馈

## License

MIT
