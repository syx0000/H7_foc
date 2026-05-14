# 万里扬 FDCAN V1.7 协议验证流程

## 静态分析已知疑点（验证前必读）

| # | 文件:行 | 问题 | 影响 |
|---|---------|------|------|
| 1 | `can_wly.c:119` | `pack_status_frame` 用 `dtheta_mech_out`（电机端 rpm×1024，不含 GR）但 `vel_int_to_rad_s` 一律 `÷(1024*GR)` | 速度反馈被缩小 25 倍 |
| 2 | `can_wly.c:455` | `CommunicateErr` 一旦置位，总线恢复后不会自动清除 | 需手动 `0x701 ... FD` 清错 |
| 3 | `can_wly.c:315` & `foc_api.c:514` | `temp1` 同时被 Rs 辨识和 node_id 占用 | bwtest3 后 ID 复位 |
| 4 | `can_wly.c:222-243` | MIT 帧的 Kp/Kd/Vel 全部丢弃，仅做位置环 + Tff | 不符合 V1.7 完整 MIT 语义 |
| 5 | `can_wly.c:126-127` | `ServoErrFlag` 32bit 截到 24bit，bit 28-30（zeroPoint/currentOffset/flashRead）丢失 | 高位故障无法上报 |

---

## 第 1 层：MCU 端单元自测（不接总线）

### 目的

不依赖 USB-CAN，直接通过 USART1 调试命令构造收帧 → 调用 `fdcan_rx_user` → 比对内部状态。能在最早期暴露字节序、单位换算、SDO 字典 bug。

### 实现步骤

1. 在 `foc/foc_fast/foc_bsp.c` 的 `dbg_cmd_set()` 里追加 `cantest<N>` 命令分支
2. 每条命令：
   - 构造一组 hex 字节
   - 调 `fdcan_rx_user(id, buf, len)` 模拟收帧
   - 通过 USART1 打印解析后的 `velocity_ref / position_ref / I_q_ref` 等内部状态
   - 对于会回响应帧的请求（SDO），把 `fdcan_send` 暂时打桩到打印 hex

### 测试用例

| 命令 | 模拟帧 | 验证点 | 期望 |
|------|--------|--------|------|
| `cantest1` | `0x200` D=`E8 03 01` (v_raw=1000, ID=1) | velocity_ref 单位换算 | 解析出 v_rad_s ≈ -16.5 rad/s, velocity_ref = vel_rad_s_to_int 结果 |
| `cantest2` | `0x400` D=`<p[24]> <v[16]> 01` | position_ref + Pid_PositionLimit | 比对 1°/1024 LSB 精确值 |
| `cantest3` | 调 `pack_status_frame` 直接打印 12 字节 | 状态帧打包 | **重点验证速度刻度疑点** |
| `cantest4` | `0x600` D=`40 00 20 00 00 00 00 00` | SDO 读 0x2000 (pos_min) | 响应 D=`60 00 20 00 <-5.0f LE>` |
| `cantest5` | `0x600` D=`23 00 20 00 <-10.0f LE>` | SDO 写 0x2000 | g_can_wly_lim.pos_min = -10.0f |
| `cantest6` | `0x701` D=`FF FF FF FF FF FF FF FA` | 控制帧使能 | foc_run = 1 |
| `cantest7` | 注入帧后停 250ms，看 1ms tick | 超时保护 | CommunicateErr 在第 200ms 置 1 |
| `cantest8` | `0x500` 12 字节 MIT | MIT 解析 | **确认 Kp/Kd/Vel 是否被处理（当前实现忽略）** |

### 通过判据

- 每条命令打印的内部状态值与协议手工换算一致（小数点后 3 位）
- 状态帧 12 字节打回后，按协议反向解码 ↔ 当前 `controller_eyou` 字段闭环

---

## 第 2 层：USB-CAN 上位机回环测试

### 硬件准备

- USB-CAN 适配器（任选）：PCAN-USB FD / 周立功 USBCANFD-200U / 创芯科技 CANFD-100U
- 双绞线 + 120Ω 终端电阻 × 2
- 波特率：Nominal **1 Mbps** / Data **5 Mbps**（FDCAN BRS）
- 节点 ID：从站 = 1（默认）

### 测试矩阵

按下表逐条发送，抓包比对：

| 测项 | 发送帧 | 期望应答 | 验证点 |
|------|--------|----------|--------|
| T1 | `0x080` (任意 8B) | `0x101` 12B 状态帧 | 广播查询 |
| T2 | `0x081` (兼容单播) | `0x101` 12B | 单播查询路径 |
| T3 | `0x601` D=`40 00 20 00 00 00 00 00` | `0x581` D=`60 00 20 00 <pos_min LE>` | SDO 读 0x2000 (ID=0x600+NodeID) |
| T4 | `0x601` D=`23 00 20 00 <-10.0f LE>` | `0x581` D=`60 ...` | SDO 写 0x2000，再读回比对 |
| T5 | `0x601` D=`23 00 2F 00 02 00 00 00` | `0x581` 应答 | 改 node_id → 2，下一帧基址切换 |
| T6 | `0x601` D=`23 05 2F 00 02 00 00 00` | `0x581` + 1ms 主动上报 | 0x2F05 自动上报 |
| T7 | `0x200` D=`<v_le> 01` | `0x101` 状态帧 | 速度模式生效，反馈跟踪 |
| T8 | `0x400` D=`<p[24]><v[16]> 01` | `0x101` | 位置模式，看 real_position_out 收敛 |
| T8b | `0x300` D=`<t[16]> 01` | `0x101` | 转矩模式，I_q_ref 跟踪 (3B/槽) |
| T9 | `0x500` D=`<MIT 12B>` | `0x101` | MIT，**确认 Kp/Kd 是否被丢弃（当前实现）** |
| T10 | `0x701` D=`FF FF FF FF FF FF FF FA` | `0x101` sta.Bit0=1 | 使能 |
| T11 | `0x701` D=`FF FF FF FF FF FF FF FB` | sta.Bit0=0, vel/iq=0 | 失能 |
| T12 | `0x701` D=`FF FF FF FF FF FF FF FC` | mech_offest_out 更新 | 置零 |
| T13 | `0x701` D=`FF FF FF FF FF FF FF FD` | ServoErrFlag.All_Flag=0 | 清错 |
| T14 | 停发 200ms | ServoErrFlag.Bit.CommunicateErr=1 | 看门狗 |
| T15 | 触发 OC/堵转 | err1/err2/sta.Bit1 正确反映 | 故障编码 |
| T16 | 加热 MOS 至 90℃ | warn.Bit0=1, sta.Bit2=1 | 温度警告 |
| T17 | 走到目标位置内 | sta.Bit3=1 | 到达位 |

### 操作步骤

1. **板子上电**，USART1 921600 抓 log 确认 `can_wly_init` 完成
2. **基线状态帧抓取**：发 T1，记录 12 字节 hex 作为基线
3. **逐条执行 T3 ~ T13**，每条留 50ms 间隔避免冲击
4. **配对测试**（写 → 读）：T4 写完立即 T3 读回比对
5. **多帧并发**：T7 持续发 50Hz × 30s，看 `s_tx_fail_count` 是否累计
6. **故障注入**（T14 ~ T17）逐项触发，记录 ServoErrFlag 和 sta 字节
7. **清错恢复**：每次故障后用 T13 清，确认 sta.Bit1 复位

### 工具建议

- **PCAN-View**：手动逐帧测试，方便对照协议 PDF
- **Python + python-can**：写 `can_wly_master.py` 跑断言测试，便于回归

```python
# 候选脚本骨架
import can
bus = can.Bus(interface='pcan', channel='PCAN_USBBUS1',
              fd=True, bitrate=1000000, data_bitrate=5000000)
def send(arb_id, data):
    msg = can.Message(arbitration_id=arb_id, data=data,
                      is_fd=len(data) > 8, bitrate_switch=len(data) > 8,
                      is_extended_id=False)
    bus.send(msg)
def recv(timeout=0.2):
    return bus.recv(timeout=timeout)
```

---

## 第 3 层：与真实主站联调

### 场景

1. **多节点共线**：从站 ID=1, 2 各一台，主站发 `0x200` 复合帧 `<v1> 01 <v2> 02`
   - 验证 `find_slot` 扫描逻辑只取自己槽位
2. **MIT PD 行为**：主站 0x500 实时下发 Kp/Kd
   - 当前实现忽略 Kp/Kd → **预期不通过**，作为 backlog 项记录
3. **长时压测**：1 小时 × 100Hz 状态查询 + 50Hz 速度指令
   - 监控：`s_tx_fail_count` / 总线 ACK 错误率 / `CommunicateErr` 触发频率
4. **冷启动一致性**：断电 → 上电后 node_id 是否从 Flash 正确恢复

### 通过标准

- 1 小时压测 `s_tx_fail_count == 0`
- 多节点正交：节点 1 不响应节点 2 的指令
- 故障 → 清错 → 恢复 流程闭环

---

## 验证产出

每层完成后在本文件追加：

- [ ] 第 1 层：cantest 命令实现 + 8 条用例全通过
- [ ] 第 2 层：T1~T17 PCAN 抓包记录归档
- [ ] 第 3 层：长时压测日志 + 多节点抓包

---

## 修复优先级（验证发现 bug 后）

| P | 问题 | 修复方向 |
|---|------|---------|
| P0 | 速度反馈刻度（疑点 #1） | 状态帧 vel 改用 `dtheta_mech_out / 1024` 直接得 rpm，再换 rad/s（不再 ÷GR） |
| P0 | node_id 与 Rs 共占 temp1（疑点 #3） | node_id 移到独立 Flash 字段 |
| P1 | CommunicateErr 不自愈（疑点 #2） | 收到有效帧时同步清除 |
| P1 | ServoErrFlag 高 8 bit 截断（疑点 #5） | 状态帧 D[9] 改成完整 `(All_Flag>>16)&0xFF` 时确认所有重要 bit 在低 24 位；或改协议扩展 |
| P2 | MIT 阉割（疑点 #4） | 实现独立 MIT_PD_MODE，在电流环前加 PD 运算 |
