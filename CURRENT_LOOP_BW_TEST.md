# 电流环带宽测试 (单频注入 + 0x7FD 上报)

## 一、原理

在 I_q 指令上叠加单频正弦扰动, 同步采集指令值与反馈值, 计算每个频点的:
- **幅值比** = |I_q| / |I_q_ref|  → 增益 dB
- **相位差** = ∠I_q - ∠I_q_ref    → 相位 deg

逐点测出 Bode 图, 取 -3dB 点 = 闭环带宽。

## 二、相关接口

### 串口 dbg 命令 (本机)

| 命令 | 说明 | 等效 CAN |
|------|------|----------|
| `testfreq<N>` | 注入频率 N Hz (1~5000) | 0x601: `23 06 2F 02 [N LSB..MSB]` |
| `testampl<N>` | 注入幅值 N Q10 (1024=1A, ≤30720=30A) | 0x601: `23 07 2F 02 [N LSB..MSB]` |
| `teststart` | 启动单频注入 + 0x7FD 数据流 | 0x601: `23 05 2F 01 01 00 00 00` |
| `teststop`  | 停止 (打印 tx_ok/tx_fail) | 0x601: `23 05 2F 01 00 00 00 00` |

### 0x7FD 帧格式 (8B, FOC ISR 10kHz 逐拍上报)

```
byte[0..3] = (int32) (I_q_ref_filterd_A * 1000) + 50000   LSB first
byte[4..7] = (int32) (I_q_A             * 1000) + 50000   LSB first
```

量程 ±50A (uint32 + 50000 偏置), 协议层已限 30A。

## 三、测试流程

### 1. 准备 (一次性)

```
Run cmd1.0 M4 tar0              # 切转矩模式
                                # 目标电流 = 0 (注入信号围绕 0 摆)
                                # 使能 PWM (foc_run=1, ISR 跑起来)
```

### 2. 设注入参数

```
testfreq1000     # 1000 Hz
testampl666
```

### 3. 启动注入 + 录数据

```
teststart
# 输出: test started: 1000 Hz, 1.00 A
# CAN 分析仪开始录 0x7FD 帧 (持续 ≥ 1秒)
```

### 4. 停止 + 检查发送计数

```
teststop
# 输出: test stopped: 0x7FD tx_ok=12345, tx_fail=0
```

**判断**:
- `tx_ok ≈ 测试时长(s) × 10000` → 正常
- `tx_ok = 0` → ISR 没跑 (没 enable1) 或 s_test_active 没置上
- `tx_fail >> tx_ok` → CAN 总线异常 (见排错章节)

### 5. 下一个频率

```
testfreq2000     # 改频率不需要 stop
teststart        # 直接重新启动 (会清零计数)
# ... 录数据 ...
teststop
```

### 6. 全部测完

```
target0
enable0          # 失能 PWM
```

## 四、推荐频点

| 段位 | 频点 (Hz) | 用途 |
|------|----------|------|
| 低频 | 10, 50, 100, 200 | 静态增益, 验证 0dB 平直段 |
| 中频 | 500, 1000, 1500 | 主调段, -3dB 点附近 |
| 高频 | 2000, 3000, 4000 | 衰减段, 看相位裕度 |

每个频点录 1~2 秒, 共采样 ≥ 10000 点。

## 五、数据后处理

### Python 解析示例

```python
import numpy as np

# 从 CAN 分析仪导出 0x7FD 帧 (CSV/MAT)
def parse_7fd(byte8):
    ref = int.from_bytes(byte8[0:4], 'little') - 50000  # mA
    fb  = int.from_bytes(byte8[4:8], 'little') - 50000
    return ref / 1000.0, fb / 1000.0  # → A

# 同步检测算单频点 Bode
def bode_point(ref, fb, f0, fs=10000):
    n = len(ref)
    t = np.arange(n) / fs
    sin_w = np.sin(2 * np.pi * f0 * t)
    cos_w = np.cos(2 * np.pi * f0 * t)

    ref_re = np.mean(ref * cos_w) * 2; ref_im = np.mean(ref * sin_w) * 2
    fb_re  = np.mean(fb  * cos_w) * 2; fb_im  = np.mean(fb  * sin_w) * 2

    H = (fb_re + 1j*fb_im) / (ref_re + 1j*ref_im)
    gain_db = 20 * np.log10(np.abs(H))
    phase_deg = np.angle(H, deg=True)
    return gain_db, phase_deg
```

## 六、CAN 物理层要求

ISR 10kHz × 8B 帧 ≈ **80KB/s**, FOC ISR 直接调 `fdcan_send`, TX FIFO 仅 10 槽。
要求:
- **CAN-FD 1M/5M BRS** (当前配置 NominalPrescaler=5, DataPrescaler=1)
- **必须有接收节点 ACK**, 否则 FIFO 永远满, tx_fail 飙升
- 推荐 CAN 分析仪: PCAN-USB FD / Kvaser / ZLG USBCANFD

## 七、排错

### 现象: tx_ok=10, tx_fail=巨大

**原因**: TX FIFO (10 槽) 一次塞满后无 ACK, 帧堵在 FIFO 出不去。

**排查**:
1. `canstat` 看协议状态:
   - `LEC=ACK_ERROR` → 没接收节点 / 总线没接
   - `Bus-Off=1` → 总线短路 / 阻抗错配
2. 确认 CAN 分析仪:
   - 已连入总线
   - 配置成 **CAN-FD BRS** (不是 Classic CAN)
   - 波特率匹配 (Nominal 1M, Data 5M)
3. 终端电阻 120Ω × 2 (总线两端各一)

### 现象: tx_ok=0

**原因**: ISR 没跑或注入未启用。

**排查**:
1. `enable1` 是否执行 (foc_run 是否=1)
2. teststart 后 `s_test_active` 是否=1
3. 模式是否在 TEST_MOTOR_CURRENT_MODE 之外但仍跑 ISR (转矩 mode3 也行)

### 现象: 0x7FD 收得到但波形畸变

**原因**: 注入幅值过大触发饱和 / I_q_ref 偏置太接近限幅。

**排查**:
1. 降幅值: testampl 改小到 0.3~0.5A
2. target 设 0 (中点), 别让基础指令偏太多
3. 看 I_q_ref_filterd 是否撞 MaxCurrent 圆限幅

## 八、与全频段扫频 bwtest1 的区别

| | `bwtest1` (TestCurrentLoopBandwidth) | `teststart` (本流程) |
|---|---|---|
| 频段 | 自动扫 10~2500Hz | 手动逐点 |
| 数据 | 串口打印每点结果 | CAN 0x7FD 逐拍原始数据 |
| 用途 | 快速看带宽 | 精细分析波形 / 自定义后处理 |
| 注入幅值 | 写死 0.3A | testampl 可调 |
| 工作偏置 | 写死 0.5A | target 可调 |

两者底层公用 `controller->bw_test` 与 `s_test_*` 状态机, 不要同时开。
