"""
can_console.py — CAN-FD 调试上位机 CLI

Phase 1: ping / version 命令.
后续 Phase 增加 logid / pid / bwtest / flash / ota 等子命令.

用法:
    python can_console.py ping
    python can_console.py version
    python can_console.py --channel 0 ping
    python can_console.py --no-filter ping       # 不设滤波器, 看万里扬流量
"""
from __future__ import annotations

import argparse
import sys
from typing import Optional

from cxcanfd_driver import CXCanFD
import can_debug_protocol as proto


def _open_bus(args) -> CXCanFD:
    bus = CXCanFD(dev_index=args.dev_index, channel=args.channel,
                  abit=args.abit, dbit=args.dbit)
    if not args.no_filter:
        # 仅接收 0x7E0~0x7EF (调试通道) 帧, 屏蔽万里扬流量
        bus.set_filter(proto.ID_CMD, 0x7EF, std=True)
    return bus


def _xchg(bus: CXCanFD, frame: bytes, expect_cmd: int, timeout_ms: int = 200):
    """发一帧, 等一帧 0x7E1 响应. 返回 CmdResp."""
    if not bus.send(proto.ID_CMD, frame):
        raise RuntimeError("CAN TX failed (FIFO full?)")
    while True:
        rx = bus.recv(timeout_ms=timeout_ms)
        if rx is None:
            raise TimeoutError(f"no resp for CMD 0x{expect_cmd:02X} within {timeout_ms}ms")
        cid, data, _ = rx
        if cid != proto.ID_RESP:
            continue  # 万里扬协议或其他帧, 忽略
        resp = proto.parse_resp(data)
        if resp.cmd == expect_cmd:
            return resp


# ===== 子命令 =====
def cmd_ping(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_ping(), proto.CMD.PING, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] PING err={resp.err.name}")
            return 1
        info = proto.parse_ping_payload(resp.payload)
        print(f"PING ok. MCU proto_ver={info.proto_ver}, host expects {proto.PROTO_VER}")
        if info.proto_ver != proto.PROTO_VER:
            print(f"[WARN] proto version mismatch: MCU={info.proto_ver}, host={proto.PROTO_VER}")
            return 2
        return 0


def cmd_version(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_version(), proto.CMD.VERSION, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] VERSION err={resp.err.name}")
            return 1
        v = proto.parse_version_payload(resp.payload)
        print(f"Firmware: SW={v.soft!r}  HW={v.hard!r}  Build={v.build!r}")
        return 0


def cmd_reset(args) -> int:
    with _open_bus(args) as bus:
        try:
            resp = _xchg(bus, proto.pack_reset(), proto.CMD.RESET, timeout_ms=args.timeout)
            if resp.ok:
                print("RESET ack received, MCU is rebooting...")
            else:
                print(f"[FAIL] RESET err={resp.err.name}")
                return 1
        except TimeoutError:
            # MCU 复位前来不及发 ACK 也算正常
            print("RESET sent (no ack, MCU likely rebooting).")
        return 0


def cmd_logid(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_logid_set(args.id), proto.CMD.LOGID_SET, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] LOGID_SET err={resp.err.name}")
            return 1
        print(f"logid set to {args.id}")
        return 0


def cmd_logfreq(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_logfreq_set(args.period_ms), proto.CMD.LOGFREQ_SET, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] LOGFREQ_SET err={resp.err.name}")
            return 1
        print(f"logfreq set to {args.period_ms} ms")
        return 0


def cmd_pid(args, loop: str) -> int:
    cmd_map = {"current": proto.CMD.CUR_PID_SET, "speed": proto.CMD.SPD_PID_SET, "position": proto.CMD.POS_PID_SET}
    cmd = cmd_map[loop]
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_pid_set(cmd, args.kp, args.ki, args.kd), cmd, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] {loop.upper()}_PID_SET err={resp.err.name}")
            return 1
        print(f"{loop} PID set: Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
        return 0


def cmd_flash_write(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_flash_write(), proto.CMD.FLASH_WRITE, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] FLASH_WRITE err={resp.err.name}")
            return 1
        print("Flash write OK")
        return 0


def cmd_flash_erase(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_flash_erase(), proto.CMD.FLASH_ERASE, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] FLASH_ERASE err={resp.err.name}")
            return 1
        print("Flash erase OK")
        return 0


def cmd_fault_clear(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_fault_clear(), proto.CMD.FAULT_CLR, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] FAULT_CLR err={resp.err.name}")
            return 1
        print("Faults cleared")
        return 0


def cmd_enable(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_enable(args.state == 1), proto.CMD.ENABLE, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] ENABLE err={resp.err.name}")
            return 1
        print(f"PWM {'enabled' if args.state else 'disabled'}")
        return 0


def cmd_phase_comp(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_phase_comp_set(args.off_pos, args.off_neg, args.comp_pos, args.comp_neg),
                     proto.CMD.PHASE_COMP_SET, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] PHASE_COMP_SET err={resp.err.name}")
            return 1
        print(f"Phase comp set: off_pos={args.off_pos}, off_neg={args.off_neg}, comp_pos={args.comp_pos}, comp_neg={args.comp_neg}")
        return 0


def cmd_phase_comp_save(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_phase_comp_save(), proto.CMD.PHASE_COMP_SAVE, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] PHASE_COMP_SAVE err={resp.err.name}")
            return 1
        print("Phase comp saved to Flash")
        return 0


def cmd_canrxdbg(args) -> int:
    with _open_bus(args) as bus:
        resp = _xchg(bus, proto.pack_canrxdbg(args.state == 1), proto.CMD.CANRXDBG, timeout_ms=args.timeout)
        if not resp.ok:
            print(f"[FAIL] CANRXDBG err={resp.err.name}")
            return 1
        print(f"CAN RX debug {'ON' if args.state else 'OFF'}")
        return 0


def cmd_log_to_csv(args) -> int:
    """订阅 0x7E2 周期日志, 写入 CSV. 假设 MCU 已经 logid <id> 设好"""
    import csv
    import time

    with _open_bus(args) as bus:
        # 切换日志 ID + 频率 (可选)
        if args.logid is not None:
            resp = _xchg(bus, proto.pack_logid_set(args.logid), proto.CMD.LOGID_SET,
                         timeout_ms=args.timeout)
            if not resp.ok:
                print(f"[FAIL] LOGID_SET err={resp.err.name}")
                return 1
        if args.logfreq is not None:
            resp = _xchg(bus, proto.pack_logfreq_set(args.logfreq), proto.CMD.LOGFREQ_SET,
                         timeout_ms=args.timeout)
            if not resp.ok:
                print(f"[FAIL] LOGFREQ_SET err={resp.err.name}")
                return 1

        print(f"Capturing 0x7E2 log frames -> {args.out} for {args.duration}s...")
        t_end = time.monotonic() + args.duration
        n_frames = 0
        n_lost = 0
        last_seq = None
        writer = None
        f = None

        try:
            f = open(args.out, 'w', newline='')
            while time.monotonic() < t_end:
                rx = bus.recv(timeout_ms=200)
                if rx is None:
                    continue
                cid, data, ts_us = rx
                if cid != proto.ID_LOG:
                    continue
                try:
                    log = proto.parse_log(data)
                except ValueError as e:
                    print(f"[WARN] parse failed: {e}")
                    continue

                # 检测丢帧 (seq 单调 +1)
                if last_seq is not None:
                    expected = (last_seq + 1) & 0xFF
                    if log.seq != expected:
                        gap = (log.seq - expected) & 0xFF
                        n_lost += gap
                last_seq = log.seq

                # 首帧建表头
                if writer is None:
                    fieldnames = ['ts_us', 'log_id', 'seq', 'ts_ms'] + list(log.fields.keys())
                    writer = csv.DictWriter(f, fieldnames=fieldnames)
                    writer.writeheader()

                row = {'ts_us': ts_us, 'log_id': log.log_id, 'seq': log.seq, 'ts_ms': log.ts_ms}
                row.update({k: str(v) for k, v in log.fields.items()})
                writer.writerow(row)
                n_frames += 1
        finally:
            if f:
                f.close()

        print(f"Captured {n_frames} frames, ~{n_lost} dropped (by seq gap)")
        if n_frames > 0:
            print(f"Rate: {n_frames / args.duration:.1f} Hz")
        return 0


def cmd_listen(args) -> int:
    """实时打印 0x7E2 / 0x7E3 帧, 不存盘. 用于 sanity check."""
    import time

    with _open_bus(args) as bus:
        print(f"Listening for {args.duration}s... (Ctrl+C to stop)")
        t_end = time.monotonic() + args.duration
        try:
            while time.monotonic() < t_end:
                rx = bus.recv(timeout_ms=500)
                if rx is None:
                    continue
                cid, data, _ = rx
                if cid == proto.ID_LOG:
                    log = proto.parse_log(data)
                    print(f"[LOG  {log.log_id:3d} seq={log.seq:3d} ts={log.ts_ms:5d}] {log.fields}")
                elif cid == proto.ID_EVENT:
                    evt = proto.parse_event(data)
                    print(f"[EVT  0x{evt.event_id:02X}] {evt.payload.hex()}")
                elif cid == proto.ID_RESP:
                    resp = proto.parse_resp(data)
                    print(f"[RESP] cmd=0x{resp.cmd:02X} ok={resp.ok}")
        except KeyboardInterrupt:
            print("\nstopped.")
        return 0


# ===== 主入口 =====
def main(argv: Optional[list] = None) -> int:
    p = argparse.ArgumentParser(description="CAN-FD 调试上位机 (创芯 USB-CAN)")
    p.add_argument("--dev-index", type=int, default=0)
    p.add_argument("--channel", type=int, default=0, help="CAN 通道 (0 或 1)")
    p.add_argument("--abit", type=int, default=1_000_000, help="仲裁波特率")
    p.add_argument("--dbit", type=int, default=5_000_000, help="数据相波特率")
    p.add_argument("--timeout", type=int, default=200, help="单次响应等待 ms")
    p.add_argument("--no-filter", action="store_true",
                   help="不设硬件滤波 (调试时看万里扬流量用)")

    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("ping", help="链路探活 + 协议版本号")
    sub.add_parser("version", help="读取固件版本")
    sub.add_parser("reset", help="远程复位 MCU")

    # 日志类
    p_logid = sub.add_parser("logid", help="切换周期日志")
    p_logid.add_argument("id", type=int, help="日志 ID (0/10/30/40/50/60/70/100/110/...)")

    p_logfreq = sub.add_parser("logfreq", help="设置日志周期")
    p_logfreq.add_argument("period_ms", type=int, help="周期 (ms)")

    # PID 类
    p_cur_pid = sub.add_parser("pid-current", help="电流环 PID")
    p_cur_pid.add_argument("kp", type=int)
    p_cur_pid.add_argument("ki", type=int)
    p_cur_pid.add_argument("kd", type=int)

    p_spd_pid = sub.add_parser("pid-speed", help="速度环 PID")
    p_spd_pid.add_argument("kp", type=int)
    p_spd_pid.add_argument("ki", type=int)
    p_spd_pid.add_argument("kd", type=int)

    p_pos_pid = sub.add_parser("pid-position", help="位置环 PID")
    p_pos_pid.add_argument("kp", type=int)
    p_pos_pid.add_argument("ki", type=int)
    p_pos_pid.add_argument("kd", type=int)

    # Flash 类
    sub.add_parser("flash-write", help="写 Flash (保存当前参数)")
    sub.add_parser("flash-erase", help="擦除 Flash")
    sub.add_parser("fault-clear", help="清除故障标志")

    # 运动控制类
    p_enable = sub.add_parser("enable", help="PWM 使能/失能")
    p_enable.add_argument("state", type=int, choices=[0, 1], help="0=失能, 1=使能")

    p_phase = sub.add_parser("phase-comp", help="设置相位补偿")
    p_phase.add_argument("off_pos", type=int, help="正转固定偏置 (×0.1°)")
    p_phase.add_argument("off_neg", type=int, help="反转固定偏置 (×0.1°)")
    p_phase.add_argument("comp_pos", type=int, help="正转速度补偿 (×0.1)")
    p_phase.add_argument("comp_neg", type=int, help="反转速度补偿 (×0.1)")

    sub.add_parser("phase-comp-save", help="保存相位补偿到 Flash")

    # CAN 状态类
    p_canrxdbg = sub.add_parser("canrxdbg", help="CAN RX 调试打印开关")
    p_canrxdbg.add_argument("state", type=int, choices=[0, 1], help="0=关, 1=开")

    # 周期日志 / 异步事件
    p_log = sub.add_parser("log-to-csv", help="抓 0x7E2 周期日志到 CSV")
    p_log.add_argument("--logid", type=int, default=None, help="先切日志 ID (省略表示沿用当前)")
    p_log.add_argument("--logfreq", type=int, default=None, help="先设日志周期 ms (省略表示沿用当前)")
    p_log.add_argument("--duration", type=float, default=10.0, help="抓取时长 (秒)")
    p_log.add_argument("--out", type=str, default="log.csv", help="输出 CSV 路径")

    p_listen = sub.add_parser("listen", help="实时打印 0x7E2/0x7E3 帧 (sanity check)")
    p_listen.add_argument("--duration", type=float, default=30.0, help="监听时长 (秒)")

    args = p.parse_args(argv)
    handlers = {
        "ping": cmd_ping,
        "version": cmd_version,
        "reset": cmd_reset,
        "logid": cmd_logid,
        "logfreq": cmd_logfreq,
        "pid-current": lambda a: cmd_pid(a, "current"),
        "pid-speed": lambda a: cmd_pid(a, "speed"),
        "pid-position": lambda a: cmd_pid(a, "position"),
        "flash-write": cmd_flash_write,
        "flash-erase": cmd_flash_erase,
        "fault-clear": cmd_fault_clear,
        "enable": cmd_enable,
        "phase-comp": cmd_phase_comp,
        "phase-comp-save": cmd_phase_comp_save,
        "canrxdbg": cmd_canrxdbg,
        "log-to-csv": cmd_log_to_csv,
        "listen": cmd_listen,
    }
    return handlers[args.cmd](args)


if __name__ == "__main__":
    sys.exit(main())
