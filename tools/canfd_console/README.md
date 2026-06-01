# CAN-FD 调试上位机 (Phase 1-3)

基于创芯 USBCANFD-2CH (`ControlCANFD.dll`) 的 Python CLI, 用于 STM32H743 调试通道 (0x7E0~0x7EF).

详细协议设计见 [`../canfd/CAN_DEBUG_DESIGN.md`](../canfd/CAN_DEBUG_DESIGN.md).

## 环境

- Windows (创芯 DLL 仅 Windows)
- Python 3.8+ 64-bit
- 无外部依赖 (纯 ctypes + 标准库)

## 文件

| 文件 | 用途 |
|------|------|
| `ControlCANFD.dll` | 创芯官方驱动 (从 `tools/canfd/` 拷贝) |
| `cxcanfd_driver.py` | DLL ctypes 封装 + `CXCanFD` 类 |
| `can_debug_protocol.py` | CMD_ID / 错误码 / 帧 pack/unpack (与 `Core/Inc/can_debug.h` 同步) |
| `can_console.py` | CLI 入口 |
| `test_protocol.py` | 协议层离线单元测试 (不需要硬件) |

## 使用

接好创芯盒子 (CAN0 接 MCU PA11/PA12 + 共地 + 120Ω 终端电阻), MCU 上电后:

```bash
cd tools/canfd_console

# 链路探活
python can_console.py ping

# 读固件版本
python can_console.py version

# 切换周期日志
python can_console.py logid 50
python can_console.py logfreq 100

# PID 在线调参
python can_console.py pid-current 45 4 0
python can_console.py pid-speed 1500 10 0
python can_console.py pid-position 3016 9 0

# PWM 使能/失能
python can_console.py enable 1
python can_console.py enable 0

# Flash 操作
python can_console.py flash-write
python can_console.py flash-erase
python can_console.py fault-clear

# 相位补偿
python can_console.py phase-comp 0 0 20 26
python can_console.py phase-comp-save

# CAN RX 调试打印
python can_console.py canrxdbg 1

# 远程复位 MCU
python can_console.py reset
```

## 自检

```bash
# 离线协议测试
python test_protocol.py

# 仅打开设备 (验证 DLL + 盒子 + 驱动通)
python cxcanfd_driver.py
```

## Phase 路线

- ✅ **Phase 1**: PING / VERSION / RESET
- ✅ **Phase 2**: 共享执行层抽取 (14 个函数)
- ✅ **Phase 3**: LOGID/LOGFREQ/PID/ENABLE/FLASH/PHASE_COMP/CANRXDBG (本次)
- ⏸ Phase 4: 周期日志 (0x7E2) + 异步事件 (0x7E3) + foc_tuner GUI 接 CAN 后端
- ⏸ Phase 5: OTA + 0x7E6 文本透传

## 注意

- 默认设硬件滤波 `0x7E0~0x7EF`, 屏蔽万里扬协议流量. 用 `--no-filter` 可关闭.
- 默认仲裁 1Mbps + 数据 5Mbps, 与 MCU `fdcan.c` 配置一致 (FDCAN kernel = 100MHz).
- `CMD_ID` 在 C/Python 两端必须同步, 改一侧记得改另一侧.
- 串口和 CAN 调试通道**完全并存**, 互不干扰, 可同时使用.
