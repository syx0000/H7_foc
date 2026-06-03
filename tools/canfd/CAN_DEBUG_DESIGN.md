# CAN-FD 调试上位机设计文档

> 状态：**草案 v0.1**，等待评审。  
> 目的：把现有 USART1 调试通道（logid 周期日志 / bwtest 辨识 / PID 在线调参 / Cali / enable / Runcmd / OTA）镜像一份到 CAN-FD，**并存**，互不干扰，便于上位机通过创芯 USB-CAN 盒子接管全部调试功能。  
> 范围：MCU 端协议栈 + Python 上位机；不包含上位机 GUI（后续按需扩展）。

---

## 0. TL;DR

- **物理层**：复用现有 FDCAN1（PA11/PA12，1 Mbps 仲裁 + 5 Mbps 数据，ISO CAN-FD），波特率与万里扬协议天生匹配，**fdcan.c 不动**。
- **协议层**：在万里扬已用 ID 段（0x080~0x77F + 0x7FD/0x7FE）之外，**另开 0x7E0~0x7EF** 作为调试/日志/OTA 专用段，类别码独立、解析独立、和万里扬零冲突。
- **MCU 端**：新增 `Core/Src/can_debug.c/h`；同时把 `foc_bsp.c::dbg_cmd_set` 中的"逻辑执行体"抽出为 `dbg_cmd_*` 共享函数（参数化、无 printf），让串口和 CAN 都能调用。串口路径继续输出 printf 文本，CAN 路径输出二进制帧。**所有现有串口命令 100% 保留，行为零变化。**
- **上位机**：基于创芯随盒提供的 `ControlCANFD.dll` + ctypes（已有 demo `cxcanfd_x64_v2.0.py`），分两块：
  - 新建 `tools/canfd_console/`（CLI，Phase 1~3 用，给 PING/版本/PID/bwtest 等命令做最小可用接口）
  - 给现有 `tools/foc_tuner/` PyQt 上位机加 CAN 后端（Phase 4~5），**复用 waveform_widget 双光标 + bode + console + 所有面板**，串口/CAN 通过下拉切换
- **不动的东西**：`MDK-ARM/*`、`fdcan.c` 波特率配置、`can_wly.c` 中已上线的协议处理、`tools/foc_tuner/` 现有面板逻辑（只在 core 层加新后端）。

---

## 1. 背景与现状

### 1.1 现有通信链路

```
┌───────────────────────────────────────────────────────────┐
│                  STM32H743 MCU                            │
│                                                           │
│  USART1 (921600, DMA)            FDCAN1 (1M+5M)           │
│   ├─ printf 日志              ├─ 万里扬协议从站           │
│   ├─ DMA IDLE RX              │  (运动控制 + SDO)         │
│   └─ dbg_cmd_set(text)        │  0x080~0x77F              │
│      └─ 调用执行函数 ◄────┐   ├─ 0x7FD 电流带宽逐拍流     │
│                           │   ├─ 0x7FE 扩展状态帧         │
│                           │   └─ (待加) 0x7E0~0x7EF 调试  │
│                           │       └─ 调用执行函数 ─┐      │
│                           │                        │      │
│                           └────────共享层───────────┘      │
└───────────────────────────────────────────────────────────┘
        ▲                                    ▲
        │ COM4                               │ USB-CAN
   ┌────┴─────┐                       ┌──────┴────────┐
   │ PyQt串口 │  (保留, 不动)         │ Python CLI    │  (新建)
   │ console  │                       │ + 创芯DLL     │
   └──────────┘                       └───────────────┘
```

### 1.2 现有调试功能盘点（要镜像到 CAN）

| 串口命令 | 功能 | 频率 | 数据流向 | 备注 |
|---------|------|------|---------|------|
| `logid <N>` | 切换周期日志（10/11/30/40/50/60/70/90/100/110/130/140/150/151） | 设置 1 次 | 主→从 | 切换后由 `dbg_log_print()` 周期发 |
| `logfreq <ms>` | 设置周期日志间隔 | 设置 1 次 | 主→从 | 默认 100ms |
| `CurrentPIDKp..Ki..Kd..` | 电流环 PID 在线调参 | 偶发 | 主↔从 | 写入后回显，写 FlashData RAM 镜像 |
| `SpeedPIDKp..Ki..Kd..` | 速度环 PID | 偶发 | 主↔从 | 同上 |
| `PositionPIDKp..Ki..Kd..` | 位置环 PID | 偶发 | 主↔从 | 同上 |
| `bwtest1`~`bwtest10` | 带宽测试 / 辨识 / autoTune / 死区标定 | 每次秒级阻塞 | 主→从→主 | **结果异步打印**，可能触发写 Flash |
| `Cali` | 电角度辨识 + 写 Flash | 阻塞 ~3s | 主→从 | 完成回 `Cali done` |
| `Runcmd<X>M<Y>tar<Z>` | 启动运行（mode + target） | 偶发 | 主→从 | 已有万里扬 0x500 MIT 替代 |
| `enable<0/1>` | PWM 使能/失能 | 偶发 | 主→从 | 已有万里扬 0x701 替代 |
| `logid 160/161/162/163` | 写 Flash / 擦 Flash / Dump 对比 / 清错 | 偶发 | 主→从 | 与 `logid` 复用，特殊值 |
| `getparams` | 一次性查询所有 PID + 相位补偿 | 偶发 | 主↔从 | 输出 `PARAMS_BEGIN..PARAMS_END` 块 |
| `offsetpos/neg`,`comppos/neg`,`savephasecomp` | 相位补偿在线调参 + 落盘 | 偶发 | 主→从 | |
| `injectV<mV>` | 开环注入电压调试 SVPWM | 阻塞 5s | 主→从 | 周期 100ms 打印 |
| `testfreq/testampl/teststart/teststop` | 单频注入 + 0x7FD 流（已对齐 SDO 0x2F0x） | 偶发 | 主→从 | **CAN 端已实现**，串口为别名 |
| `canstat` | FDCAN 状态查询 + bus-off 重置 | 偶发 | 主↔从 | |
| `cantest<N>` | 万里扬协议自测（Stub） | 偶发 | 本地 | 调试用，CAN 上位机不需要 |
| `mit<N>` | MIT 测试序列（Stub） | 偶发 | 本地 | 同上 |
| `otabegin/otaend/otaabort/otaswap` | OTA 升级控制 | 串行 ~MB 级 | 主→从 | 大数据量 |
| `version`,`reset` | 版本查询 / 系统复位 | 偶发 | 主↔从 | |

**结论**：约 30+ 命令需要镜像。其中 `cantest` / `mit` 是协议 stub 自测，与 CAN 上位机无关；`Runcmd` / `enable` 万里扬已覆盖；其余都需要新通道。

### 1.3 已确认参数

| 项目 | 值 | 来源 |
|------|-----|-----|
| FDCAN kernel clock | **100 MHz** | `main.c::PeriphCLKInit`：HSE=25M / PLL2M=5 / PLL2N=100 / PLL2Q=5 |
| 仲裁波特率 | **1 Mbps** (SP 85%) | `fdcan.c`：100M/5/(1+16+3) |
| 数据相波特率 | **5 Mbps** (SP 85%) | `fdcan.c`：100M/1/(1+16+3) |
| 帧格式 | FD_BRS（标准 ID 11 位 + 64B 数据） | `fdcan.c::FrameFormat = FDCAN_FRAME_FD_BRS` |
| RX FIFO | FIFO0，10 槽，每槽 32B | `fdcan.c::RxFifo0ElmtsNbr=10, ElmtSize=32` |
| TX FIFO | 10 槽，每槽 32B | `fdcan.c::TxFifoQueueElmtsNbr=10, TxElmtSize=32` |
| 滤波器 | mask=0 / id=0，**接收所有标准帧** | `fdcan.c::sFilterConfig` |
| 中断优先级 | FDCAN1_IT0 = 6 | `fdcan.c::HAL_NVIC_SetPriority` |
| 创芯盒子 | USBCANFD-2CH（VCI_USBCAN2=41） | `tools/canfd/python.../cxcanfd_x64_v2.0.py:8` |
| 创芯 DLL | `ControlCANFD.dll`（API `ZCAN_*`） | `tools/canfd/ControlCANFD.dll` |

⚠️ TX 槽 32B 限制了**单帧最大调试 payload 32 字节**——超过这个量必须分片。这是关键约束（见 §3.2）。

---

## 2. 设计决策（头脑风暴产物）

### 2.1 共存策略 — 串口和 CAN 完全独立、共享底层

**决策**：两套**前端解析器**（USART1 文本解析 + FDCAN1 二进制解析）共用一套**执行函数**。
- 串口路径：`USART1 RX → dbg_cmd_set() → dbg_cmd_apply_*() → printf 文本反馈`
- CAN 路径：`FDCAN1 RX → can_debug_handle_cmd() → dbg_cmd_apply_*() → fdcan_send 二进制响应帧`

**为什么不复用万里扬 SDO（0x600/0x580）扩展对象字典**：
- SDO 单帧 8B 载荷无法承载周期日志（需要 64B 帧）。
- 调试命令大多是"动作触发"语义（bwtest3、Cali、teststart），不是"读写参数"，硬塞进对象字典会让索引爆炸（0x3000~0x4FFF 都得占）。
- 万里扬协议是工业上线要兼容的，里面塞调试命令污染主协议，不利于以后给客户对接。
- 调试帧 ID 段独立，**上位机可以一行 ZCAN_SetFilterStartID/EndID(0x7E0,0x7EF) 把万里扬流量过滤掉**，调试期间观察更清晰。

### 2.2 ID 段分配 — 为什么选 0x7E0~0x7EF

万里扬已用：`0x080 / 0x100~0x17F / 0x200 / 0x300 / 0x400 / 0x500 / 0x580~0x5FF / 0x600~0x67F / 0x700~0x77F / 0x7FD / 0x7FE`。

11 位标准 ID 范围 0x000~0x7FF，剩下的连续空段：
- `0x000~0x07F`（128 个，但靠近 0x080 万里扬段，将来扩位置广播容易撞）
- **`0x7E0~0x7EF`**（16 个，孤立、对齐 16 边界、远离万里扬段、便于范围滤波）✓
- `0x7FF`（单个，传统留作"管理员"，不建议占）

**16 个 ID 够用吗**：调试通道实际只需要 6 个，留 10 个给 OTA / 未来扩展（高速波形流、多通道并发）。

| ID | 方向 | 用途 | 频率 | 帧大小 |
|----|------|------|------|--------|
| **0x7E0** | PC→MCU | 调试命令请求 | 偶发 | 1~32B |
| **0x7E1** | MCU→PC | 调试命令响应 | 偶发 | 2~32B |
| **0x7E2** | MCU→PC | 周期日志流 | 高速（最高 1kHz） | 8~64B |
| **0x7E3** | MCU→PC | 异步事件（辨识完成、故障、Cali done） | 偶发 | 8~64B |
| **0x7E4** | PC→MCU | OTA 数据分片 | 串行 | 64B |
| **0x7E5** | MCU→PC | OTA 进度/ACK | 串行 | 8~16B |
| **0x7E6** | MCU→PC | 调试文本透传（可选，把 printf 复制到 CAN） | 偶发 | 1~64B |
| 0x7E7~0x7EF | - | 预留 | - | - |

### 2.3 命令编码 — 1 字节命令码 + 类型化负载

**决策**：每个 0x7E0 帧的 `Byte[0]` 是 **CMD_ID**（命令枚举），后续字节按命令的 schema 解析。所有 schema 在 `can_debug.h` 里**用结构体 + `__attribute__((packed))` 表达**，不写自由格式协议表，让 C 编译器和 Python `struct` 模块各自校验。

避免的反模式：
- ❌ 文本 over CAN（`"logid 50"` 直接塞进帧）：上位机要写两套解析（CAN/串口），且失去 CAN 的强类型优势。
- ❌ 复用 SDO 索引（每个命令一个 OD 项）：见 §2.1。
- ❌ 多字段位域打包（节省 1B 但调试地狱）：现在 64B 帧充裕，对齐字节边界优先。

**CMD_ID 分组**（高 4 bit 是大类）：

```
0x0_  系统类     0x00=NOP/PING, 0x01=VERSION, 0x02=RESET, 0x03=GET_PARAMS
0x1_  日志类     0x10=LOGID_SET, 0x11=LOGFREQ_SET, 0x12=LOG_PRINT_TEXT(透传开关)
0x2_  PID 类     0x20=CUR_PID_SET, 0x21=SPD_PID_SET, 0x22=POS_PID_SET, 0x23=PID_GET_ALL
0x3_  辨识/带宽  0x30=BWTEST(子命令1~10), 0x31=CALI, 0x32=INJECT_V, 0x33=DEAD_TIME
0x4_  Flash      0x40=FLASH_WRITE, 0x41=FLASH_ERASE, 0x42=FLASH_DUMP, 0x43=FAULT_CLR
0x5_  运动控制   0x50=ENABLE, 0x51=RUN_CMD, 0x52=PHASE_COMP_SET, 0x53=PHASE_COMP_SAVE
0x6_  CAN 状态   0x60=CAN_STAT, 0x61=CAN_RX_DBG_TOGGLE
0x7_  注入测试   0x70=TEST_FREQ, 0x71=TEST_AMPL, 0x72=TEST_START, 0x73=TEST_STOP
0x8_  OTA        0x80=OTA_BEGIN, 0x81=OTA_END, 0x82=OTA_ABORT, 0x83=OTA_SWAP
0xFE  保留       
0xFF  错误响应（仅 0x7E1 用）
```

### 2.4 周期日志 — 二进制结构体 vs 文本透传

**决策**：**二进制结构体为主（0x7E2），文本透传为辅（0x7E6）**。
- 0x7E2：每个 LOG_ID 一个 packed struct，上位机用 Python `struct.unpack` 解码，**带宽小、解析快、可绘图**。
- 0x7E6：把 `printf` 的字节流原样塞进 CAN 帧，**当串口没接的时候**给上位机看 stdout。这条不上来就做，留接口，等需要时打开。

**为什么二进制**：
- USART1 文本流 921600 bps ≈ 92KB/s；
- 0x7E2 64B 帧 1kHz = 64KB/s（接近上限）；
- 二进制载荷比文本紧凑 3~5 倍 → 同等带宽支持更高频日志。

**周期日志触发位置**：复用现有 `dbg_log_print()` 的调用栈（main 循环每 `logPriodMs` 触发一次），但在 `dbg_log_print()` 内部根据 `dbgLogFlag` **同时**走 printf（串口）和 `can_debug_send_log()`（CAN）。两路都受 `logPriodMs` 节流，互不阻塞。

### 2.5 异步事件 — bwtest 完成、故障、Cali done

`bwtest1` 等命令是**阻塞秒级**的：在 ISR 里没法跑（会 watchdog 喂不饱），目前的实现是在主循环里同步执行，期间 USART1 还能继续打文本。

CAN 路径同理：
- 命令到达 → 0x7E1 立刻回 ACK（"已开始"）→ 命令在主循环同步执行 → 完成后发 0x7E3（"完成 + 结果"）。
- 上位机看到 0x7E1 ACK 后**不阻塞**，继续轮询 0x7E3。

**0x7E3 帧格式**：`[EVENT_ID, STATUS, payload...]`，比如 bwtest3 完成发：
```c
struct {
    uint8_t  event_id;      // = 0x30 (BWTEST_DONE)
    uint8_t  test_index;    // = 3
    uint8_t  status;        // 0=OK, 非0=错误码
    float    rs_ohm;
    float    ld_h;
    float    lq_h;
    uint8_t  reserved[5];   // padding to 20B
} __attribute__((packed));
```

### 2.6 错误处理与超时

- **未知 CMD_ID**：0x7E1 回 `[0xFF, CMD_ID, 0x01_UNKNOWN]`。
- **参数超界**：0x7E1 回 `[0xFF, CMD_ID, 0x02_OUT_OF_RANGE]`。
- **设备繁忙**（bwtest 在跑时收到第二个 bwtest）：回 `[0xFF, CMD_ID, 0x03_BUSY]`。
- **CAN 调试通道独立超时**：调试帧**不喂万里扬看门狗**（`s_can_timeout_cnt`），避免上位机长时间不发命令把万里扬协议的运动控制误超时停机。
- **TX FIFO 满**：fdcan_send 返回 ERROR，上层维护 drop counter，不阻塞、不重试（重试在调试场景没意义，关键命令上位机会重发）。

### 2.7 安全约束

| 风险 | 对策 |
|------|------|
| CAN 通道触发 `Runcmd` / `enable` 等动作类命令时，万里扬协议正在控运动 | 调试通道命令命中 `0x5_` 大类时，**先检查 controller_mode**，若 ≠ IDLE/TORQUE_REF=0 则拒绝（同万里扬 ENABLE 检查 fault_brake_is_active） |
| OTA 写 Flash 期间被打断 | OTA Stage1 已有的状态机沿用，`g_ota_rx_mode` 一旦激活，0x7E0 普通命令暂停响应（除 `OTA_ABORT`） |
| 上位机断线导致电机失控 | **不引入 CAN 调试通道的运动看门狗**（运动安全是万里扬负责的事）。调试通道掉线 = 调试中断，运动控制不受影响 |
| 大端/小端 | 全部 **little-endian**（与 STM32 native 一致，省去转换） |
| float 跨平台 | C 端 `union { uint32_t u; float f; }`，Python `struct.pack('<f', ...)`，IEEE 754 单精度 |

### 2.8 CMD echo 约定

`Byte[0]` 在请求和响应中**始终回显 CMD_ID**，错误响应用 `0xFF` 顶格，把原 CMD_ID 放 `Byte[1]`：

```
请求:    [CMD_ID][...args]
成功响应: [CMD_ID][0x00][...payload]
错误响应: [0xFF][CMD_ID][ERR_CODE][...optional_detail]
```

上位机靠 `Byte[0]==0xFF` 单字节判定走错误分支，简化解析。

---

## 3. 详细协议规范

### 3.1 帧 Layout 总览（再贴一次）

```
0x7E0 (PC→MCU)  请求      [CMD_ID][args 0..31]
0x7E1 (MCU→PC)  响应      [CMD_ID][STATUS=0][payload]   或   [0xFF][CMD_ID][ERR]
0x7E2 (MCU→PC)  周期日志   [LOG_ID][SEQ][TS_LO][TS_HI][payload]   total ≤ 64B
0x7E3 (MCU→PC)  异步事件   [EVENT_ID][...]
0x7E4 (PC→MCU)  OTA 数据   [SEQ_LO][SEQ_HI][...60B chunk]
0x7E5 (MCU→PC)  OTA ACK    [SEQ_LO][SEQ_HI][STATUS][CRC32...]
```

### 3.2 单帧 32B 上限（来自 TX FIFO 配置）— 最终决策：保持 32B

`fdcan.c` 配置 `TxElmtSize = FDCAN_DATA_BYTES_32`，即每个发送槽**32 字节**。

> ⚠️ **实测教训**：升到 FDCAN_DATA_BYTES_64 会导致 Message RAM 超限，万里扬协议帧收发异常（电机抖动/失控）。**永远不改 FIFO 大小**。

**最终约束：所有调试帧 payload ≤ 32B，超过的拆帧或裁剪字段。**

实际场景适配：
- 0x7E1 响应：VERSION 帧裁剪到 32B（soft 10B + hw 8B + build 11B + 头 2B = 31B）✓
- 0x7E2 日志：最长 logid=40（4B header + 7×4B = 32B）刚好踩线 ✓
- 0x7E2 logid=110（ADC ISR 分段耗时 12×2B = 24B + 4B header = 28B）✓
- 0x7E3 事件：bwtest 完成最大 20B ✓
- 0x7E4 OTA：**改为 28B chunk**（2B seq + 28B data = 30B），速率降低但安全

### 3.3 命令字典（带 schema）

> 下表 schema 用 C 风格描述，全部 `__attribute__((packed))` little-endian。
> "args 长度" 包含 CMD_ID 字节本身。

#### 0x0_ 系统类

```
0x00 PING              args[1]: [CMD_ID]
                       resp[2]: [CMD_ID][0x00]
                       用途: 上位机连接确认

0x01 VERSION           args[1]: [CMD_ID]
                       resp[20]: [CMD_ID][0x00][soft_ver:u16][hard_ver:u16]
                                  [build_date:char[8]][build_time:char[6]]
                       用途: 替代 "version" 串口命令

0x02 RESET             args[1]: [CMD_ID]
                       resp: 无 (复位前发不出来)
                       用途: 替代 "reset" 串口命令

0x03 GET_PARAMS        args[1]: [CMD_ID]
                       resp[28]:
                         [CMD_ID][0x00]
                         [cur_kp:u32][cur_ki:u32][cur_kd:u32]   // 12B
                         [spd_kp:u32][spd_ki:u32][spd_kd:u32]   // 12B  → 26B 超 32 单帧?
                       *** 改为分组返回: 0x03 sub=1/2/3 各回一组,避免单帧>20B ***

0x03 GET_PARAMS v2     args[2]: [CMD_ID][group: 1=cur, 2=spd, 3=pos, 4=phase_comp]
                       resp[14]: [CMD_ID][0x00][group][kp:u32][ki:u32][kd:u32]
                                 (group=4 时载荷不同, 见 0x52)
```

#### 0x1_ 日志类

```
0x10 LOGID_SET         args[3]: [CMD_ID][log_id:u16]   // 0/10/11/30/40/50/60/70/90/100/110/130/140/150/151
                       resp[3]: [CMD_ID][0x00][log_id_echo:u16]

0x11 LOGFREQ_SET       args[3]: [CMD_ID][period_ms:u16]
                       resp[3]: [CMD_ID][0x00][period_ms_echo:u16]

0x12 LOG_TEXT_TOGGLE   args[2]: [CMD_ID][enable:u8]    // 0=关 0x7E6 文本透传, 1=开
                       resp[2]: [CMD_ID][0x00]
```

#### 0x2_ PID 类

```
0x20 CUR_PID_SET       args[13]: [CMD_ID][kp:u32][ki:u32][kd:u32]
                       resp[14]: [CMD_ID][0x00][kp_echo][ki_echo][kd_echo]
                       行为: 同 "CurrentPIDKp..Ki..Kd..", 同时写 IncPID_QAxis/DAxis + FlashData

0x21 SPD_PID_SET       同上, IncPID_Speed + FlashData
0x22 POS_PID_SET       同上, IncPID_Position + FlashData
0x23 PID_GET_ALL       已合并到 0x03 GET_PARAMS
```

#### 0x3_ 辨识/带宽

```
0x30 BWTEST            args[2]: [CMD_ID][test_idx:u8]   // 1~10
                       resp 立刻回: [CMD_ID][0x00][test_idx]   // ACK "已开始"
                       异步 0x7E3: [0x30][test_idx][status][payload]
                          test_idx=3 (motor_param):
                              payload: [rs:f32][ld:f32][lq:f32]                   = 12B
                          test_idx=4 (flux):
                              payload: [psi_f:f32]                                = 4B
                          test_idx=5 (inertia):
                              payload: [J:f32]                                    = 4B
                          test_idx=6 (autoTune cur):
                              payload: [kp:u32][ki:u32]                           = 8B
                          test_idx=7 (autoTune spd): 同上
                          test_idx=8 (autoTune pos): 同上
                          test_idx=1/2/9 (带宽):
                              payload: [bw_hz:f32][peak_db:f32][pm_deg:f32]       = 12B

0x31 CALI              args[1]: [CMD_ID]
                       resp 立刻: [CMD_ID][0x00]
                       异步 0x7E3: [0x31][status][payload?]   // status: 0=ok, 1=erase_fail

0x32 INJECT_V          args[5]: [CMD_ID][mv:i32]
                       resp 立刻: [CMD_ID][0x00]
                       周期 0x7E3 (50 次, 每 100ms): [0x32][idx][i_a_q10:i32][i_d_q10:i32][r_a_x100:u32][r_d_x100:u32]

0x33 DEAD_TIME         args[1]: [CMD_ID]
                       同 BWTEST=10
```

#### 0x4_ Flash

```
0x40 FLASH_WRITE       args[1]: [CMD_ID]
                       resp[2]: [CMD_ID][0x00]
0x41 FLASH_ERASE       args[1]: [CMD_ID]
                       resp[2]: [CMD_ID][status]   // 0=OK, 1=fail
0x42 FLASH_DUMP        args[2]: [CMD_ID][page:u8]   // 0=header, 1=PID, 2=offsets, 3=motor_params
                       resp[最大32B]: [CMD_ID][0x00][page][...]
                       (因为 FlashSavedData 太大, 分页返回)
0x43 FAULT_CLR         args[1]: [CMD_ID]
                       resp[2]: [CMD_ID][0x00]
```

#### 0x5_ 运动控制（已有万里扬替代，仅保留为调试快捷方式）

```
0x50 ENABLE            args[2]: [CMD_ID][enable:u8]
0x51 RUN_CMD           args[10]: [CMD_ID][cmd_val:u8][mode_val:u8][reserved:u8][tar:f32]
0x52 PHASE_COMP_SET    args[9]: [CMD_ID][off_pos:i16][off_neg:i16][comp_pos:i16][comp_neg:i16]
0x53 PHASE_COMP_SAVE   args[1]: [CMD_ID]
```

#### 0x6_ CAN 状态

```
0x60 CAN_STAT          args[1]: [CMD_ID]
                       resp[24]: [CMD_ID][0x00]
                                 [tx_err:u8][rx_err:u8][bus_off:u8][err_passive:u8][warning:u8]
                                 [last_err:u8][data_last_err:u8][activity:u8]
                                 [tx_fifo_free:u8][tx_fail_cnt:u32]
                                 [node_id:u8][reserved:u8]
                       副作用: 如果 BusOff=1 或 tx_fifo_free=0, 自动 reset peripheral
0x61 CAN_RX_DBG_TOGGLE args[2]: [CMD_ID][enable:u8]
```

#### 0x7_ 注入测试（与万里扬 SDO 0x2F0x 共享底层状态）

```
0x70 TEST_FREQ         args[5]: [CMD_ID][hz:u32]
0x71 TEST_AMPL         args[5]: [CMD_ID][q10:u32]
0x72 TEST_START        args[1]: [CMD_ID]
                       resp[2]: [CMD_ID][0x00]
                       后续: 已有 0x7FD 流自动产生 (调用 can_wly_test_start)
0x73 TEST_STOP         args[1]: [CMD_ID]
                       resp[10]: [CMD_ID][0x00][tx_ok:u32][tx_fail:u32]
```

#### 0x8_ OTA

```
0x80 OTA_BEGIN         args[13]: [CMD_ID][size:u32][crc:u32][ver:u32]
                       resp[2]: [CMD_ID][0x00]   // 后续 g_ota_rx_mode=1
0x81 OTA_END           args[1]: [CMD_ID]
                       resp[6]: [CMD_ID][status:u8][crc_calc:u32]
0x82 OTA_ABORT         args[1]: [CMD_ID]
0x83 OTA_SWAP          args[1]: [CMD_ID]
                       resp 无 (复位)

OTA 数据流: 0x7E4 [seq:u16][...60B chunk]    上位机推
            0x7E5 [seq:u16][status:u8][crc_running:u32]   MCU 应答
```

### 3.4 周期日志 schema（0x7E2）

```c
// 通用头 (4B)
struct can_log_hdr_t {
    uint8_t  log_id;       // = LOGID 当前值
    uint8_t  seq;           // 单调递增, 用于检测丢帧
    uint16_t timestamp_ms;  // HAL_GetTick() 低 16 位
};

// LOGID=10 Angle_elec
struct log_10_t {
    can_log_hdr_t h;
    int32_t now_mechposition;
    uint16_t theta_elec;
    int32_t real_position_out;
    int32_t real_position;
    int32_t dtheta_mech_div1024;
};  // 4 + 4 + 2 + 4 + 4 + 4 = 22B

// LOGID=40 Current PI
struct log_40_t {
    can_log_hdr_t h;
    int32_t I_q, I_d, V_q, V_d;
    int32_t I_q_ref, I_d_ref, I_q_ref_filterd;
};  // 4 + 7*4 = 32B

// LOGID=50 Speed
struct log_50_t {
    can_log_hdr_t h;
    int32_t v_ref_rpm;
    int32_t v_ref_filt_rpm;
    int32_t v_fb_motor_rpm;
    int32_t v_fb_load_rpm;
    int32_t v_err_rpm;
};  // 4 + 5*4 = 24B

// LOGID=70 CCR + Iabc
struct log_70_t {
    can_log_hdr_t h;
    uint16_t CCR2, CCR3, CCR4;
    int32_t  I_a, I_b, I_c;
};  // 4 + 6 + 12 = 22B

// LOGID=110 ADC ISR timing (last + max, 单位 us)
struct log_110_t {
    can_log_hdr_t h;
    uint16_t tot, tot_max;
    uint16_t read, read_max;
    uint16_t enc, enc_max;
    uint16_t pos, pos_max;
    uint16_t vel, vel_max;
    uint16_t cur, cur_max;
};  // 4 + 12*2 = 28B
```

所有 LOGID 的 packed struct 集中放在 `can_debug_log.h`，C 端编译时静态断言 `sizeof <= 64`，Python 端 `struct.calcsize('<' + fmt)` 校验对齐。

---

## 4. MCU 端实现计划

### 4.1 文件改动

| 文件 | 动作 | 说明 |
|------|------|------|
| `Core/Inc/can_debug.h` | **新增** | CMD_ID 枚举、错误码、log struct、API 声明 |
| `Core/Src/can_debug.c` | **新增** | 命令派发 + 0x7E2 周期日志发送 + 0x7E3 异步事件 |
| `Core/Src/can_wly.c` | **小改** | `fdcan_rx_user` 在万里扬分发末尾加 `0x7E0/0x7E4` 旁路到 `can_debug_*` |
| `Core/Src/fdcan.c` | **不改** | 保持 32B FIFO，不动 Message RAM 布局 |
| `foc/foc_fast/foc_bsp.h` | **小改** | 暴露执行函数签名（见 §4.2） |
| `foc/foc_fast/foc_bsp.c` | **重构** | `dbg_cmd_set` 内部逻辑抽成 `dbg_cmd_apply_*` 共享函数；`dbg_log_print` 调用增加 `can_debug_send_log()` 旁路 |
| `Core/Src/main.c` | **1 行** | `can_wly_init()` 后加 `can_debug_init()` |
| `MDK-ARM/cubemx_yxsui.uvprojx` | **加入新源文件** | 让 Keil 编译 `can_debug.c` |

### 4.2 共享执行函数抽取（foc_bsp 重构）

把 `dbg_cmd_set()` 里**每个命令**的逻辑抽成独立函数，命名规则 `dbg_cmd_apply_<name>`，**不在函数里 printf**（printf 由调用者按通道决定）：

```c
// 在 foc_bsp.h 暴露:
typedef enum { CMD_OK = 0, CMD_ERR_BUSY, CMD_ERR_FAULT, CMD_ERR_RANGE } cmd_result_t;

cmd_result_t dbg_cmd_apply_logid(uint16_t log_id);
cmd_result_t dbg_cmd_apply_logfreq(uint16_t period_ms);
cmd_result_t dbg_cmd_apply_cur_pid(uint32_t kp, uint32_t ki, uint32_t kd);
cmd_result_t dbg_cmd_apply_spd_pid(uint32_t kp, uint32_t ki, uint32_t kd);
cmd_result_t dbg_cmd_apply_pos_pid(uint32_t kp, uint32_t ki, uint32_t kd);
cmd_result_t dbg_cmd_apply_enable(uint8_t en);
cmd_result_t dbg_cmd_apply_run(int cmd_val, int mode_val, float tar_value);
cmd_result_t dbg_cmd_apply_phase_comp(int16_t off_pos, int16_t off_neg,
                                      int16_t comp_pos, int16_t comp_neg);
cmd_result_t dbg_cmd_apply_phase_comp_save(void);
cmd_result_t dbg_cmd_apply_inject_v(int32_t mv);   // 注意: 这个会阻塞 5s, 调用前要确认
cmd_result_t dbg_cmd_apply_cali(void);              // 阻塞 ~3s
cmd_result_t dbg_cmd_apply_bwtest(uint8_t idx);     // 阻塞秒级
cmd_result_t dbg_cmd_apply_flash_write(void);
cmd_result_t dbg_cmd_apply_flash_erase(void);
cmd_result_t dbg_cmd_apply_fault_clear(void);
cmd_result_t dbg_cmd_apply_canstat(can_stat_snapshot_t *out);
// ...

// 串口路径:
void dbg_cmd_set(void) {
    if (strstr(buf, "logid")) {
        uint16_t id = atoi(token);
        dbg_cmd_apply_logid(id);
        printf("logid:%d\r\n", id);   // 文本反馈仍由 dbg_cmd_set 负责
    }
    // ...
}

// CAN 路径:
void can_debug_handle_cmd(const uint8_t *data, uint32_t len) {
    if (data[0] == CMD_LOGID_SET) {
        uint16_t id = data[1] | (data[2] << 8);
        cmd_result_t r = dbg_cmd_apply_logid(id);
        can_debug_send_resp(CMD_LOGID_SET, r, &id, 2);
    }
    // ...
}
```

**重构原则**：
1. **执行函数纯计算/纯副作用**，不打印、不格式化字符串。
2. **错误状态用枚举返回**，调用者决定如何反馈（printf vs CAN 帧）。
3. **现有串口行为零变化** — 重构后 USART1 上看到的输出必须和重构前一致。每改一个命令就 build + 抓串口 log 对照（`tools/capture_com.ps1`）。
4. **拆解粒度**：每个 `dbg_cmd_apply_*` 函数 ≤ 30 行，复杂的（bwtest 子分发）继续拆。

⚠️ 阻塞类命令（bwtest / Cali / inject_v）现在是**主循环里同步执行**的。CAN 分发也只能在主循环（不能在 FDCAN1_IT0 ISR 里阻塞秒级）。**约束**：`fdcan_rx_user` 里收到 0x7E0 后**只入队**，主循环 poll 队列再调 `dbg_cmd_apply_*`。

### 4.3 ISR / 主循环职责划分

```
FDCAN1_IT0 ISR (优先级 6)
└─ HAL_FDCAN_RxFifo0Callback
    └─ fdcan_rx_user(id, data, len)
        ├─ id 在万里扬段 → can_wly 内部处理 (现有, 短小, 可以在 ISR 里做)
        └─ id == 0x7E0 → can_debug_enqueue(data, len)   // 仅入队, 立即返回
        └─ id == 0x7E4 → ota_data_chunk_rx(data, len)   // OTA 数据流, 入 OTA ring buf

main loop
└─ can_debug_poll()
    └─ 出队一帧 → can_debug_dispatch(cmd) → dbg_cmd_apply_xxx() → 发 0x7E1 响应
└─ dbg_cmd_set()                  // 串口命令, 不变
└─ dbg_log_print()                // 周期日志
    ├─ printf 文本 (USART1, 不变)
    └─ can_debug_send_log()       // 新增, 走 0x7E2
```

**入队队列**：环形缓冲 8 槽 × 64B（与 RX FIFO 等深），SPSC，ISR 写、main 读。和现有 `can_dbg_buf` 类似，但单独维护（避免和万里扬调试缓冲混用）。

### 4.4 串口和 CAN 双路日志同步策略

当前 `dbg_log_print()` 由 main 循环调用，节流逻辑（`logPriodMs`）已经在 `dbg_cmd_log_print` 内做。改造后：

```c
void dbg_log_print(void) {
    // 节流判断 (现有, 不变)
    if (HAL_GetTick() - last < logPriodMs) return;
    last = HAL_GetTick();

    switch (dbgLogFlag) {
    case 50:
        printf("speed: %d, %d, ...\r\n", ...);   // 串口路径 (不变)
        can_debug_send_log_50();                  // CAN 路径 (新增)
        break;
    // ...
    }
}
```

CAN 一路如果 TX FIFO 满就丢帧（计数到 `s_can_log_drop_cnt`，由 `0x60 CAN_STAT` 命令查询），不阻塞串口。

### 4.5 fdcan.c — 不改动

> ⚠️ 实测确认：**fdcan.c 的 FIFO 元素大小保持 FDCAN_DATA_BYTES_32 不变**。
> 升到 64B 会导致 Message RAM 超限，万里扬帧收发异常。
> 所有调试帧 payload ≤ 32B。OTA 用 28B chunk 而非 60B。

### 4.6 编译验证

每改一个命令做一次：
1. `cmd.exe /c '"C:\Keil_v5\UV4\UV4.exe" -b "MDK-ARM\cubemx_yxsui.uvprojx" -j0'`
2. 看 `MDK-ARM/cubemx_yxsui/cubemx_yxsui.build_log.htm` 0 Error 0 Warning
3. （可选）烧录 + 串口抓 log 对照重构前

---

## 5. Python 上位机实现计划

### 5.1 项目结构

```
tools/canfd_console/
├── README.md
├── ControlCANFD.dll                  ← 创芯 DLL (从 tools/canfd/ 拷贝)
├── cxcanfd_driver.py                 ← 创芯 ctypes 封装 (基于 demo 改)
├── can_debug_protocol.py             ← CMD_ID 枚举 + struct 定义 (与 can_debug.h 一一对应)
├── can_console.py                    ← 主入口 (CLI)
├── examples/
│   ├── set_pid.py                    ← 调 PID 例子
│   ├── log_to_csv.py                 ← 抓 0x7E2 周期日志存 CSV
│   ├── run_bwtest.py                 ← 跑 bwtest3 + 等 0x7E3 结果
│   └── ota_upload.py                 ← Stage1 上传固件
└── tests/
    └── test_protocol.py              ← 帧打包/解包单元测试 (不需要硬件)
```

### 5.2 创芯 DLL 封装层

直接复用 demo `cxcanfd_x64_v2.0.py` 的 ctypes 结构体定义，**抽出成模块**：

```python
# cxcanfd_driver.py
class CXCanFD:
    def __init__(self, dev_index=0, ch=0):
        self.dll = windll.LoadLibrary('./ControlCANFD.dll')
        # ...同 demo
        self.dev = self.dll.ZCAN_OpenDevice(VCI_USBCAN2, dev_index, 0)
        self.dll.ZCAN_SetAbitBaud(self.dev, ch, 1_000_000)
        self.dll.ZCAN_SetDbitBaud(self.dev, ch, 5_000_000)
        self.dll.ZCAN_SetCANFDStandard(self.dev, ch, 0)  # ISO
        # ...
        self.ch = self.dll.ZCAN_InitCAN(self.dev, ch, byref(cfg))
        self.dll.ZCAN_StartCAN(self.ch)

    def send_fd(self, can_id: int, data: bytes):
        # 把 bytes 封到 ZCAN_TransmitFD_Data, 调 ZCAN_TransmitFD
        ...

    def recv_fd(self, timeout_ms: int = 100) -> Optional[Tuple[int, bytes]]:
        # 调 ZCAN_GetReceiveNum + ZCAN_ReceiveFD
        ...

    def set_filter(self, start_id: int, end_id: int):
        self.dll.ZCAN_ClearFilter(self.ch)
        self.dll.ZCAN_SetFilterMode(self.ch, 0)  # 0=标准帧
        self.dll.ZCAN_SetFilterStartID(self.ch, start_id)
        self.dll.ZCAN_SetFilterEndID(self.ch, end_id)
        self.dll.ZCAN_AckFilter(self.ch)
```

**关键设计**：
- 默认调试模式只接收 `0x7E0~0x7EF` 段（屏蔽万里扬流量）。
- 扩展帧 `eff` 标志位置 0（用标准 11 位 ID）。
- BRS 标志位置 1（启用数据相 5M）。

### 5.3 协议层（Python 端镜像 can_debug.h）

```python
# can_debug_protocol.py
import struct
from enum import IntEnum

class CMD(IntEnum):
    PING            = 0x00
    VERSION         = 0x01
    RESET           = 0x02
    GET_PARAMS      = 0x03
    LOGID_SET       = 0x10
    LOGFREQ_SET     = 0x11
    CUR_PID_SET     = 0x20
    SPD_PID_SET     = 0x21
    POS_PID_SET     = 0x22
    BWTEST          = 0x30
    CALI            = 0x31
    # ...

class ERR(IntEnum):
    OK              = 0
    UNKNOWN_CMD     = 1
    OUT_OF_RANGE    = 2
    BUSY            = 3
    FAULT_ACTIVE    = 4
    BRAKE_ACTIVE    = 5

# Pack helpers
def pack_pid_set(cmd: CMD, kp: int, ki: int, kd: int) -> bytes:
    return struct.pack('<BIII', cmd, kp, ki, kd)

# Unpack log frames
LOG_FORMATS = {
    10:  '<BBHIHIII',     # log_10_t
    40:  '<BBHIIIIIII',   # log_40_t
    50:  '<BBHIIIII',     # log_50_t
    # ...
}

def unpack_log(data: bytes):
    log_id = data[0]
    fmt = LOG_FORMATS.get(log_id)
    if fmt is None:
        return None
    return struct.unpack(fmt, data[:struct.calcsize(fmt)])
```

### 5.4 CLI 接口

```bash
# 初始化 + ping
python can_console.py ping
> Connected. Firmware: soft=1.2.3, hard=1.0, build=2026-06-01 10:30

# 切日志
python can_console.py logid 50

# 设置 PID
python can_console.py pid current 45 4 0
python can_console.py pid speed 1500 10 0

# 跑带宽测试 + 等结果
python can_console.py bwtest 3
> bwtest3 started, waiting for 0x7E3...
> [BWTEST_DONE] idx=3 status=0 Rs=0.0794 Ld=0.1133mH Lq=0.1163mH (12.5s)

# 抓周期日志到 CSV
python can_console.py log-to-csv --logid 50 --duration 60 --out speed.csv

# OTA
python can_console.py ota upload firmware_app_v123.bin
```

CLI 用 `argparse` 子命令，每个动作映射到 `dbg_cmd_apply_*` 的 CAN 调用。

### 5.5 测试策略

- **离线单元测试**：`pytest tests/test_protocol.py`，只测帧 pack/unpack 一致性，不需要硬件。
- **回环测试**：创芯盒子有 2 通道（CAN0/CAN1），CAN0 发 CAN1 收，验证 ctypes 收发能跑通（demo 已经实现）。
- **MCU 联调**：先用 `ping` / `version` 验证链路，再逐个验证命令。每个命令的成功标准是**串口和 CAN 两路输出语义一致**。

### 5.6 GUI（foc_tuner 加 CAN 后端，复用现有面板）

> **决策修订**：原 §8 写"不在 CAN 通道实现波形显示 GUI"，已改为**做**。但**不新建独立 GUI**，而是给现有 `tools/foc_tuner/` 加 CAN 后端，复用所有面板。

#### 现有 foc_tuner 结构盘点

```
tools/foc_tuner/
├── core/
│   ├── serial_worker.py       ← 串口收发线程
│   ├── parser.py              ← 文本日志解析 (logid 50/40/...)
│   ├── protocol.py            ← 串口命令拼接
│   ├── data_model.py          ← 数据模型 + Qt 信号 (供 waveform 订阅)
│   ├── units.py               ← 单位换算
│   └── ota_worker.py          ← Stage1 OTA 上传
└── gui/
    ├── main_window.py         ← 主窗口 + Tab 容器
    ├── serial_panel.py        ← 串口连接面板
    ├── console_widget.py      ← 命令行 + 日志显示
    ├── waveform_widget.py     ← 波形 + 双光标 + Clear/Save Log
    ├── bode_widget.py         ← Bode 图 (bwtest 结果)
    ├── pid_panel.py           ← PID 在线调参
    ├── flash_panel.py         ← Flash 读写
    ├── motor_control_panel.py ← Runcmd / enable
    ├── bandwidth_test_panel.py← bwtest1~10
    ├── fault_panel.py         ← 故障显示 / 清错
    ├── maintenance_panel.py   ← Cali / inject_v / phase_comp
    └── (无 ota_panel？OTA 由 console + ota_worker 联动)
```

#### CAN 后端集成方案

**关键洞察**：所有 GUI 面板都通过 `data_model` 中的 Qt 信号订阅数据，与具体传输介质（串口/CAN）解耦。只需在 core 层加一个**与 serial_worker 同接口**的 can_worker，再在 serial_panel 加个连接选项卡，**所有 GUI 面板原封不动复用**。

```
┌──────────────────────────────────────────────────────┐
│ foc_tuner (PyQt)                                     │
│                                                      │
│  ┌─────────┐  ┌──────────┐  ┌──────────┐            │
│  │waveform │  │  bode    │  │  PID     │ ...        │
│  │widget   │  │  widget  │  │  panel   │            │
│  └────┬────┘  └────┬─────┘  └────┬─────┘            │
│       │            │              │                  │
│       └────────────┴──────────────┘                  │
│                    │                                 │
│              ┌─────▼──────┐                          │
│              │ data_model │  ← Qt 信号集线器          │
│              └─────▲──────┘                          │
│                    │                                 │
│         ┌──────────┴──────────┐                      │
│         │                     │                      │
│  ┌──────▼───────┐    ┌────────▼──────┐               │
│  │serial_worker │    │  can_worker   │ ◄─ 新增       │
│  │ + parser     │    │  + can_parser │               │
│  │ (text logid) │    │  (binary 7E2) │               │
│  └──────────────┘    └────────┬──────┘               │
│                               │                      │
└───────────────────────────────┼──────────────────────┘
                                │
                       ┌────────▼────────┐
                       │ cxcanfd_driver  │ (ctypes)
                       │ + protocol.py   │
                       └─────────────────┘
```

#### 新增/改动清单

| 文件 | 动作 | 说明 |
|------|------|------|
| `tools/foc_tuner/core/can_worker.py` | **新增** | 与 `serial_worker.py` 同 Qt 信号接口（`new_log_data` / `new_param` / `connection_changed`），底层调 `cxcanfd_driver` |
| `tools/foc_tuner/core/can_parser.py` | **新增** | 把 0x7E2 二进制 log 帧 unpack 后 emit 与 `parser.py` 同样的信号（让 waveform_widget 完全无感切换） |
| `tools/foc_tuner/core/can_protocol.py` | **新增** | CMD_ID 拼帧（与 `protocol.py` 同方法签名，比如 `set_pid(loop, kp, ki, kd)`） |
| `tools/foc_tuner/core/cxcanfd_driver.py` | **新增** | 创芯 DLL ctypes 封装（与 `tools/canfd_console/` 共用，可考虑提到上层 `tools/_shared/`） |
| `tools/foc_tuner/gui/serial_panel.py` | **小改** | 顶部加 Tab：[串口] / [CAN]，CAN 选项卡里选择 CAN 通道（CAN0/CAN1）+ 节点 ID + Connect 按钮 |
| `tools/foc_tuner/main.py` | **小改** | 启动时根据 `foc_tuner_config.json` 决定默认 worker，并维护一个 active_worker 引用 |
| `tools/foc_tuner/foc_tuner_config.json` | **小改** | 增加 CAN 配置段（盒子型号、通道、节点 ID） |

#### data_model 解耦

现有 `data_model.py` 应该已经把 logid → Qt 信号的映射做好了（如 `signal_speed_log = pyqtSignal(int, int, int, int, int)` 对应 logid 50）。

CAN 后端只要确保**emit 的信号名和参数顺序与串口一致**，所有 GUI 都自动工作。如果 data_model 里写死了"从 parser.py 接收"，需要重构为通用 emitter（这是常规代码改动，不是设计问题）。

#### 共享数据

- **CMD_ID 枚举 + log struct**：从 `tools/canfd_console/can_debug_protocol.py` 直接 import，避免两份。
- **创芯驱动**：把 `cxcanfd_driver.py` 提到 `tools/_shared/cxcanfd_driver.py`，foc_tuner 和 canfd_console 都从同一处 import。

#### GUI 双后端的实际价值

- **串口**：调试时方便，带 IDLE RX、921600 够用、文本可读、有现成 PuTTY/串口助手兜底。
- **CAN**：
  - 客户现场没串口接口（生产板 USART1 没引出）
  - 二进制日志带宽更大，**1kHz 周期日志波形更细腻**（串口文本流压不到 1ms 周期）
  - 多设备并行（一根总线挂 N 个驱动器，上位机轮询每个 NodeID）
  - OTA 速度更快（1M 仲裁 + 5M 数据 vs 串口 921600）

---


## 6. 实施分阶段计划

### Phase 1（基础设施）✅ 已完成

- [x] **P1.1** ~~改 `fdcan.c`~~ → **不改 fdcan.c**，保持 32B FIFO（实测 64B 导致 Message RAM 超限）
- [x] **P1.2** 新建 `Core/Inc/can_debug.h`：CMD_ID 枚举 + 错误码 + 函数声明
- [x] **P1.3** 新建 `Core/Src/can_debug.c`：SPSC 环形队列 + PING / VERSION / RESET + 全部命令处理
- [x] **P1.4** `can_wly.c::fdcan_rx_user` 加 0x7E0~0x7EF 旁路；`main.c` 接入
- [x] **P1.5** Python CLI：`cxcanfd_driver.py` + `can_debug_protocol.py` + `can_console.py`
- [x] **P1.6** Build: 0 Error 0 Warning（**fdcan.c 不动，万里扬协议零影响**）

> **架构决策**：can_debug.c 自包含所有 CAN 命令逻辑（直接操作 controller_eyou），
> 不依赖 foc_bsp.c 共享函数。理由：避免侵入已稳定的串口路径，降低回归风险。
> MCU 端改动仅 3 个文件：can_debug.c/h（新增）+ can_wly.c（+5行旁路）+ main.c（+2行）。

### Phase 2（共享执行层）⚠️ 已回退

> **实测教训**：把 foc_bsp.c 命令执行逻辑抽成共享函数后导致电机抖动。
> 根因是 fdcan.c FIFO 升 64B 导致 Message RAM 超限，万里扬帧收发异常。
> 排查过程中 foc_bsp.c 改动被全部回退。
>
> **最终决策**：foc_bsp.c **不做重构**，CAN 命令在 can_debug.c 中直接操作
> controller_eyou 全局结构（与串口路径独立，互不影响）。

### Phase 3（CAN 命令铺面）✅ 已完成（自包含在 can_debug.c）

- [x] CAN 端实现 LOGID/LOGFREQ/PID/FLASH/FAULT_CLR/ENABLE（直接操作 controller_eyou）
- [x] Python CLI 扩展到 15 个子命令 + 21 个单元测试通过
- [x] **不依赖 foc_bsp.c 共享函数**，零侵入串口路径

### Phase 4（周期日志 + 异步事件 + GUI 集成）✅ 已完成

- [x] **P4.1** `can_debug_send_log()` 实现 LOG_ID 10/30/40/50/60/70/90/100（全部 ≤ 32B）
- [x] **P4.2** TX FIFO 节流（< 4 槽时丢日志，保万里扬协议帧优先）
- [x] **P4.3** `can_debug_send_event()` 异步事件 API
- [x] **P4.4** Python `parse_log` / `parse_event` + LOG_ID schema
- [x] **P4.5** CLI `log-to-csv` + `listen` 子命令
- [x] **P4.6** **foc_tuner 加 CAN 后端**：`core/can_worker.py` (mimics SerialWorker)
- [x] **P4.7** **foc_tuner GUI 改造**：`serial_panel.py` 加 Backend 下拉 + `main_window.py` active_worker 切换

### Phase 4.5（MCU 补全）✅ 已完成（2026-06-03）

- [x] `foc_bsp.c::dbg_log_print()` 入口加 `can_debug_send_log()` 调用（CAN 双路输出）
  - 在 8 个常用 logid 的 printf 后面添加调用：10/30/40/50/60/70/90/100
  - 添加 `#include "can_debug.h"`
- [x] `can_debug.c` dispatch 加 PHASE_COMP_SET / PHASE_COMP_SAVE / CANRXDBG 命令处理
  - `h_phase_comp_set()`: 操作 `g_theta_offset_pos/neg`, `g_theta_comp_pos/neg`
  - `h_phase_comp_save()`: 调用 `SavePhaseCompToFlash()`
  - `h_canrxdbg()`: 操作 `g_can_rx_debug`
- [x] `can_debug_send_log()` 补全 logid 30/60/90/100
  - 添加 `w_u32()` 辅助函数
- [x] Python `can_debug_protocol.py` VERSION 响应解析适配 29B 格式（soft:10 + hw:8 + build:11）
  - 修复 `parse_version_payload()` 和单元测试

**验证结果**：
- ✅ 编译：0 Error 0 Warning
- ✅ 周期日志：logid=50, 100ms 周期，5秒 48 条无丢帧
- ✅ 命令：14/14 实现完成（PING/VERSION/RESET/LOGID/LOGFREQ/3×PID/3×FLASH/ENABLE/2×PHASE_COMP/CANRXDBG）
- ✅ 串口路径：零影响，所有 printf 保持不变

### Phase 5（OTA + 调试文本透传 + GUI 收尾）⏸ 待实施

- [ ] **P5.1** CAN 端 OTA 通道：0x7E4 数据（28B chunk）+ 0x7E5 ACK
- [ ] **P5.2** 上位机 `ota upload` CLI 实现 + 端到端测试
- [ ] **P5.3** 0x12 LOG_TEXT_TOGGLE + 0x7E6 文本透传（默认关，命令开启）
- [ ] **P5.4** **foc_tuner OTA 面板适配 CAN**：`core/ota_worker.py` 抽接口让串口/CAN 都能调
- [ ] **P5.5** **foc_tuner console_widget 接 0x7E6**：CAN 模式下也能看到 printf 输出
- [ ] **P5.6** **里程碑**：foc_tuner 在 CAN 模式下功能完整 = 串口模式

### Phase 6（收尾）⏸ 待实施

- [ ] **P6.1** `cantest` 风格的协议自测（C 端 stub 模式 + Python `pytest`）
- [ ] **P6.2** 文档：在 CLAUDE.md 增加 §"CAN 调试协议"
- [ ] **P6.3** 性能验证：1kHz 日志带宽占用、CAN 总线 utilization、TX FIFO 高水位
- [ ] **P2.3** 抽 `dbg_cmd_apply_phase_comp / phase_comp_save / fault_clear`
- [ ] **P2.4** 抽 `dbg_cmd_apply_flash_write/erase` 和 `dbg_cmd_apply_inject_v`
---

## 7. 风险与开放问题

| 风险 | 影响 | 缓解 |
|------|------|------|
| ⚠️ **FIFO 升 64B 导致 Message RAM 超限** | **万里扬帧收发异常，电机抖动/失控** | **永远不改 fdcan.c FIFO 大小**。所有调试帧 ≤ 32B。已实测确认。 |
| `fdcan.c` 被 CubeMX 重新生成 | 理论上不影响（我们不改 fdcan.c） | 无需保护 |
| 阻塞类命令（bwtest/Cali/inject_v）在主循环里跑，期间 CAN 命令积压 | 上位机超时重发 → 队列溢出 | CAN 端入队前判断 busy，回 ERR_BUSY；上位机命令文档明确"开始后等 0x7E3" |
| 1kHz 周期日志可能挤掉万里扬协议帧（共享 TX FIFO） | 状态帧延迟 / 丢帧 | TX FIFO 满时优先级：万里扬协议帧 > 调试日志帧。日志侧加 `if (HAL_FDCAN_GetTxFifoFreeLevel < 4) return;` |
| CAN 上位机意外断线 | 万里扬协议看门狗误超时？ | 调试通道独立计数，**不喂万里扬看门狗**，已在 §2.6 决策 |
| 创芯盒子在 Linux 用不了 | 跨平台调试受限 | 已确认 Windows-only，不留 driver 抽象层 |
| OTA 28B chunk（原 60B 因 32B 约束降级） | OTA 速率降低 ~50% | 仍远快于串口 921600；OTA 期间自动 `LOGID=0` 关闭周期日志减少竞争 |
| foc_bsp.c 重构导致串口命令异常 | 电机抖动 | **不重构 foc_bsp.c**。CAN 命令在 can_debug.c 独立实现，零侵入串口路径。 |

### 待用户确认的开放问题

> **状态：已确认 (2026-06-01)**

1. **OTA 第一阶段就做。** Phase 5 保留，与调试通道一起上线。
2. **0x7E6 文本透传：做。** MCU 端在 USART1 printf 路径上分一路到 CAN 环形缓冲，main loop 批量发 0x7E6。开关由 `0x12 LOG_TEXT_TOGGLE` 控制，**默认关**（避免上电就刷 CAN 总线）。
3. **评审节奏：Phase 1 跑通后停下评审一次。** PING/VERSION 链路打通 + 上位机能识别版本号后暂停，实地验证协议无问题再继续 Phase 2~5。
4. **跨平台：Windows-only。** 直接 ctypes 调创芯 DLL，不引入 driver 抽象层、不引入 python-can 依赖，代码量最小。

---

## 8. 不做的事（Out of Scope）

- ❌ **不修改 `fdcan.c` 的任何配置**（FIFO 大小、波特率、滤波器 — 实测改 FIFO 导致 Message RAM 超限）
- ❌ **不重构 `foc_bsp.c`**（不抽共享执行层 — CAN 命令在 can_debug.c 独立实现，避免侵入串口路径）
- ❌ **不修改万里扬协议任何已上线行为**（`can_wly.c` 仅在 `fdcan_rx_user` 加 5 行旁路）
- ❌ **不引入 SocketCAN / python-can / cantools 的依赖**
- ❌ **不实现 cantest 那种 stub 自测在 CAN 通道**（用 Python `pytest` 在 host 端做）

---

## 9. 设计评审 Checklist

实现状态确认：

- [x] CMD_ID 编码无歧义（高 4 位分类清晰）
- [x] 所有命令 schema 定义在 .h 中且可被 Python 镜像
- [x] 错误响应统一 `[0xFF][CMD_ID][ERR_CODE]` 格式
- [x] 没有命令需要单帧 > 64B（FIFO 已升 64B）
- [x] 阻塞命令在 ISR 里只入队、不执行
- [x] 万里扬协议 0x080~0x77F + 0x7FD/0x7FE 与调试 0x7E0~0x7EF **无 ID 重叠**
- [x] `fdcan.c` 改动有保护机制防 CubeMX 覆盖（.ioc 已同步 + USER CODE 注释）
- [x] 串口和 CAN **共享同一套** `dbg_cmd_apply_*` 函数，无两份逻辑
- [x] Build: 0 Error 0 Warning
- [x] Python 离线测试: 21/21 通过

---

## 附录 A：与现有 `can_wly.c` 的协同点

| 共享资源 | 调用关系 |
|---------|---------|
| `controller_eyou` 全局结构 | CAN debug 路径只读 + 通过 `dbg_cmd_apply_*` 写，不直接修改 |
| `g_cantest_stub` | CAN debug 不使用（cantest 是万里扬协议自测，与调试通道无关） |
| `s_test_freq_hz` / `s_test_ampl_q10` / 0x7FD 数据流 | CAN debug 0x70~0x73 直接调用 `can_wly_set_test_freq/ampl/start/stop`，不复制状态 |
| `s_can_timeout_cnt` 万里扬看门狗 | **不**被 CAN debug 帧重置（避免上位机长时间不发命令时关电机） |
| `fdcan_send` | 共用 |
| `fdcan_rx_user` | 在万里扬段 ID 之后加 0x7E0/0x7E4 旁路 `if (id == 0x7E0) { can_debug_enqueue(...); return; }` |

## 附录 B：协议版本管理

`0x01 VERSION` 响应中带 **协议版本号** 字段（独立于固件版本）：
- 协议版本 = `1.0`（首次发布）
- 后续每次新增 CMD_ID 或修改帧 schema → 协议小版本 +1
- 删除/重命名 CMD_ID → 协议大版本 +1
- 上位机连接后比对协议版本，不兼容直接报错退出

---

## 10. 验证方案

### 10.1 硬件准备

| 项目 | 要求 |
|------|------|
| MCU 板 | STM32H743 + PA11/PA12 引出 CAN-H/CAN-L + GND |
| 创芯盒子 | USBCANFD-2CH，CAN0 通道接 MCU |
| 终端电阻 | 120Ω 跨接 CAN-H / CAN-L（两端各一个，或盒子内置跳线） |
| 串口 | USART1 同时接 PC（用于对照验证），921600 bps |
| 电源 | 48V 母线 + 电机（bwtest / enable 测试需要） |

### 10.2 验证步骤（Phase 1~4 全覆盖）

#### Step 0：烧录固件

```bash
cd C:\Users\yx\Desktop\cubemx_yxsui
cmd.exe /c '"C:\Keil_v5\UV4\UV4.exe" -f "MDK-ARM\cubemx_yxsui.uvprojx"'
```

确认串口上电打印 `FW SW=... HW=... build ...`，万里扬协议正常（cantest1 等）。

#### Step 1：CAN 链路探活

```bash
cd tools/canfd_console
python can_console.py ping
```

**预期**：`PING ok. MCU proto_ver=1, host expects 1`

```bash
python can_console.py version
```

**预期**：`Firmware: SW='20260528.1' HW='20260528' Build='Jun  1 2026'`

**失败排查**：
- 无响应 → 检查 CAN-H/CAN-L 接线、终端电阻、盒子通道号
- 协议版本不匹配 → MCU 和 Python 端 `CAN_DBG_PROTO_VER` / `PROTO_VER` 不一致

#### Step 2：命令通道验证

```bash
# 切日志 + 设频率
python can_console.py logid 50
python can_console.py logfreq 100

# PID 在线调参 (读 → 改 → 读)
python can_console.py pid-current 45 4 0
# 串口同时发 "CurrentPID" 确认值已改

# PWM 使能 (⚠️ 确保电机安全)
python can_console.py enable 1
# 串口发 "logid40" 确认电流环在跑
python can_console.py enable 0

# Flash 操作
python can_console.py flash-write
# 串口发 "logid162" 确认 RAM vs Flash 一致

# 清错
python can_console.py fault-clear

# 相位补偿
python can_console.py phase-comp 10 -10 20 26
python can_console.py phase-comp-save
```

**预期**：每条命令无 `[FAIL]` 输出，串口对照确认参数已生效。

#### Step 3：周期日志验证

```bash
# 实时监听 (先在串口发 "logid50" 确认有数据)
python can_console.py listen --duration 10
```

**预期**：每 100ms 打印一行 `[LOG  50 seq=... ts=...] {v_ref_rpm: ..., ...}`

```bash
# 抓 CSV
python can_console.py log-to-csv --logid 50 --logfreq 100 --duration 10 --out speed.csv
```

**预期**：
- `speed.csv` 约 100 行（10s × 10Hz）
- 丢帧数 ≈ 0（seq gap = 0）
- 用 Excel 打开确认数据合理

#### Step 4：高频日志压力测试

```bash
python can_console.py log-to-csv --logid 40 --logfreq 1 --duration 5 --out stress.csv
```

**预期**：
- 1kHz 日志 → 5000 帧 / 5s
- 丢帧率 < 5%（TX FIFO 节流正常工作）
- 万里扬协议同时正常（串口发 `cantest1` 确认速度指令仍能下发）

#### Step 5：异步事件验证

```bash
# 终端 1: 监听事件
python can_console.py listen --duration 30

# 终端 2 (串口): 发 bwtest3
# 或 CAN 端 (后续 Phase 实现 bwtest CAN 命令后):
# python can_console.py bwtest 3
```

**预期**：listen 终端打印 `[EVT  0x30] 03...`（bwtest3 完成事件）

#### Step 6：foc_tuner GUI 验证

```bash
cd tools/foc_tuner
python main.py
```

1. 顶部 Backend 下拉选 **CAN-FD**
2. CH=0, Arb=1000000, Data=5000000
3. 点 **Connect**
4. 左侧 Motor Control 面板切 logid=50
5. **预期**：右侧 waveform_widget 显示速度波形，双光标可拖动
6. 切 logid=40 → 电流波形
7. PID 面板改参数 → 确认波形响应变化
8. 点 **Disconnect** → 波形停止

#### Step 7：串口并存验证

同时接串口 + CAN：
1. foc_tuner Backend=CAN-FD 连接
2. 串口助手（PuTTY / 串口 PyQt console）同时打开
3. 串口发 `logid50` → 串口看到文本日志 + CAN 看到二进制日志
4. CAN 发 `pid-current 100 10 0` → 串口发 `CurrentPID` 确认值已改
5. **预期**：两路互不干扰，数据一致

#### Step 8：万里扬协议无回归

```bash
# 串口发 cantest1~7 (万里扬协议自测)
cantest1
cantest2
cantest3
...
cantest7
```

**预期**：所有 cantest 输出与改动前一致（velocity_ref / position_ref / mode 等值不变）。

### 10.3 自动化测试（离线，不需要硬件）

```bash
cd tools/canfd_console
python test_protocol.py
```

**预期**：`Ran 21 tests in 0.001s — OK`

### 10.4 Build 验证

```bash
cd C:\Users\yx\Desktop\cubemx_yxsui
cmd.exe /c '"C:\Keil_v5\UV4\UV4.exe" -b "MDK-ARM\cubemx_yxsui.uvprojx" -j0'
grep "Error\|Warning" MDK-ARM/cubemx_yxsui/cubemx_yxsui.build_log.htm
```

**预期**：`0 Error(s), 0 Warning(s)`

### 10.5 验证通过标准

| 项目 | 通过标准 |
|------|---------|
| PING/VERSION | 响应正确，协议版本匹配 |
| 命令通道 | 15 个子命令全部无 FAIL，串口对照一致 |
| 周期日志 | 10Hz 无丢帧，1kHz 丢帧 < 5% |
| 异步事件 | bwtest/Cali 完成后 listen 能捕获 |
| foc_tuner GUI | CAN 模式波形正常，双光标可用 |
| 串口并存 | 两路同时工作互不干扰 |
| 万里扬无回归 | cantest1~7 输出不变 |
| Build | 0 Error 0 Warning |
| 离线测试 | 21/21 通过 |

### 10.6 已知限制（验证时注意）

- **创芯盒子必须 Windows**：Linux/macOS 无法运行 `ControlCANFD.dll`
- **CAN 通道不喂万里扬看门狗**：如果万里扬主站在跑，CAN 调试帧不会延长超时计数器
- **阻塞命令期间 CAN 命令积压**：bwtest/Cali 执行期间（秒级），CAN 命令队列最多 8 帧，超出丢弃
- **foc_tuner CAN 后端的 `send` 翻译**：目前只支持最常用的串口命令格式，复杂命令（如 `Runcmd1M3tar1.5`）暂不支持

