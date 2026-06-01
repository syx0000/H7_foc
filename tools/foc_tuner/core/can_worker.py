"""CAN-FD worker thread for FOC motor controller (mimics SerialWorker interface).

把创芯 USBCANFD-2CH 的二进制帧"翻译"成与 USART1 文本协议同格式的字符串行,
让现有 parser.py / data_model / 所有 GUI 面板无感切换.

设计要点:
  1. 同接口同信号: sig_line_received / sig_error / sig_connected (与 SerialWorker 一致)
  2. send(cmd): 把文本命令翻译成 0x7E0 二进制帧 (logid/PID/enable/...)
  3. 接收: 0x7E2 周期日志解码 -> mimic printf 文本; 0x7E1 响应 -> mimic 文本; 0x7E3 事件忽略或转文本

依赖: tools/canfd_console/ 下的 cxcanfd_driver.py + can_debug_protocol.py
为避免重复代码, 通过相对路径 import.
"""

import os
import re
import sys
import time
from PySide6.QtCore import QThread, Signal, QMutex

# 把 tools/canfd_console 加到 sys.path, 复用 cxcanfd_driver + can_debug_protocol
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_CANFD_CONSOLE = os.path.normpath(os.path.join(_THIS_DIR, '..', '..', 'canfd_console'))
if _CANFD_CONSOLE not in sys.path:
    sys.path.insert(0, _CANFD_CONSOLE)

from cxcanfd_driver import CXCanFD  # type: ignore
import can_debug_protocol as proto  # type: ignore


class CanWorker(QThread):
    """QThread for CAN-FD communication, mimicking SerialWorker interface.

    Reads CAN frames in a loop, translates 0x7E2 binary log frames into
    text lines matching firmware printf format, then emits via sig_line_received.
    """

    sig_line_received = Signal(str)   # Translated text line (no \r\n)
    sig_error = Signal(str)           # Error message
    sig_connected = Signal(bool)      # Connection state changed

    def __init__(self, parent=None):
        super().__init__(parent)
        self._bus: CXCanFD | None = None
        self._running = False
        self._mutex = QMutex()

    def connect_port(self, channel: int = 0, abit: int = 1_000_000, dbit: int = 5_000_000):
        """Open CAN-FD device.

        Args:
            channel: 0 or 1 (USBCANFD-2CH)
            abit:  arbitration bit rate
            dbit:  data bit rate
        """
        try:
            self._bus = CXCanFD(channel=channel, abit=abit, dbit=dbit)
            # 限制接收 0x7E0~0x7EF, 屏蔽万里扬流量
            self._bus.set_filter(proto.ID_CMD, 0x7EF, std=True)
            self._running = True
            self.start()
            self.sig_connected.emit(True)
        except Exception as e:
            self.sig_error.emit(f"Failed to open CAN ch={channel}: {e}")
            self.sig_connected.emit(False)

    def disconnect_port(self):
        self._running = False
        self.wait(1000)
        if self._bus:
            try:
                self._bus.close()
            except Exception:
                pass
            self._bus = None
        self.sig_connected.emit(False)

    def send(self, cmd: str):
        """Translate text command -> 0x7E0 frame (thread-safe).

        Currently supports the most common dbg_cmd_set commands. Unknown commands
        are silently dropped (with an error signal) to avoid crashing the GUI.
        """
        self._mutex.lock()
        try:
            if not self._bus:
                return
            cmd = cmd.strip().rstrip('\r\n')
            frame = self._translate_cmd(cmd)
            if frame is None:
                self.sig_error.emit(f"CAN: unsupported cmd {cmd!r}")
                return
            self._bus.send(proto.ID_CMD, frame)
        except Exception as e:
            self.sig_error.emit(f"CAN send error: {e}")
        finally:
            self._mutex.unlock()

    # ---------- 命令文本 -> 0x7E0 二进制 ----------
    def _translate_cmd(self, cmd: str) -> bytes | None:
        """串口文本命令 -> CAN 二进制帧. 返回 None 表示不支持."""
        # logid<N>
        m = re.match(r'logid\s*(\d+)$', cmd)
        if m:
            return proto.pack_logid_set(int(m.group(1)))

        # logfreq<N>
        m = re.match(r'logfreq\s*(\d+)$', cmd)
        if m:
            return proto.pack_logfreq_set(int(m.group(1)))

        # CurrentPIDKp<a>Ki<b>Kd<c>
        m = re.match(r'(Current|Speed|Position)PIDKp(\d+)Ki(\d+)Kd(\d+)$', cmd)
        if m:
            loop, kp, ki, kd = m.groups()
            cmd_id = {'Current': proto.CMD.CUR_PID_SET,
                      'Speed':   proto.CMD.SPD_PID_SET,
                      'Position': proto.CMD.POS_PID_SET}[loop]
            return proto.pack_pid_set(cmd_id, int(kp), int(ki), int(kd))

        # enable<0/1>
        m = re.match(r'enable\s*([01])$', cmd)
        if m:
            return proto.pack_enable(m.group(1) == '1')

        # offsetpos / offsetneg / comppos / compneg / savephasecomp
        m = re.match(r'offsetpos\s*(-?\d+)$', cmd)
        if m:
            return proto.pack_phase_comp_set(int(m.group(1)), 0, 0, 0)
        # 注: 单独设一个轴的相位补偿需要先读其他三个值, 简化起见只支持
        # savephasecomp 一次性下发四个值的场景, 其他三个先用 0 (后续可加 GET_PARAMS 改进)

        if cmd == 'savephasecomp':
            return proto.pack_phase_comp_save()

        # canrxdbg<0/1>
        m = re.match(r'canrxdbg\s*([01])$', cmd)
        if m:
            return proto.pack_canrxdbg(m.group(1) == '1')

        # 简单命令
        if cmd == 'reset':
            return proto.pack_reset()
        if cmd == 'version':
            return proto.pack_version()

        # logid 160/161/163 经常被 GUI 用作"立即动作" (写/擦/清错)
        # 这些已经被 logid<N> 路径处理了 (MCU 端 dbg_log_print 的 case 160/161/163)
        # 但走 CAN 时 dbgLogFlag 设了, 主循环 dbg_log_print 也会执行, OK

        return None

    # ---------- 接收循环: 0x7E1/0x7E2/0x7E3 -> 文本 ----------
    def run(self):
        """收 CAN 帧, 翻译成文本行 emit."""
        while self._running:
            try:
                rx = self._bus.recv(timeout_ms=100) if self._bus else None
            except Exception as e:
                self.sig_error.emit(f"CAN recv error: {e}")
                time.sleep(0.05)
                continue
            if rx is None:
                continue
            cid, data, _ = rx
            try:
                line = self._translate_rx(cid, data)
            except Exception as e:
                self.sig_error.emit(f"CAN parse error: {e} ({data.hex()})")
                continue
            if line:
                self.sig_line_received.emit(line)

    def _translate_rx(self, cid: int, data: bytes) -> str | None:
        """Convert binary CAN frame -> printf-style text line."""
        if cid == proto.ID_LOG:
            return self._log_to_text(data)
        if cid == proto.ID_RESP:
            resp = proto.parse_resp(data)
            if resp.cmd == proto.CMD.VERSION and resp.ok:
                v = proto.parse_version_payload(resp.payload)
                return f"FW SW={v.soft} HW={v.hard} build {v.build}"
            # 普通 ACK 不输出文本 (避免刷屏)
            if resp.ok:
                return None
            return f"[CAN ERR] cmd=0x{resp.cmd:02X} err={resp.err.name if resp.err else '?'}"
        if cid == proto.ID_EVENT:
            evt = proto.parse_event(data)
            if evt.event_id == proto.EVT_BWTEST_DONE and evt.payload:
                return f"bwtest{evt.payload[0]}: done"
            if evt.event_id == proto.EVT_CALI_DONE and evt.payload:
                status = evt.payload[0]
                return "Cali done" if status == 0 else "Cali: Flash erase FAIL"
        return None

    # 0x7E2 二进制 -> firmware printf 同格式文本
    # parser.py 里 register 的 prefix 必须严格匹配 (Angle_elec_360: / current_pi: / speed: ...)
    @staticmethod
    def _log_to_text(data: bytes) -> str | None:
        log = proto.parse_log(data)
        f = log.fields
        if log.log_id == 10:
            return f"Angle_elec_360: {f['now_mech']}, {f['theta_elec']}, {f['pos_out']}, {f['pos']}, {f['dtheta_div1024']}"
        if log.log_id == 30:
            return f"current_get: {f['V_q']},{f['V_d']}"
        if log.log_id == 40:
            return (f"current_pi: {f['I_q']}, {f['I_d']}, {f['V_q']}, {f['V_d']}, "
                    f"{f['I_q_ref']}, {f['I_d_ref']}, {f['I_q_ref_filterd']}")
        if log.log_id == 50:
            return (f"speed: {f['v_ref_rpm']}, {f['v_ref_filt_rpm']}, "
                    f"{f['v_fb_motor_rpm']}, {f['v_fb_load_rpm']}, {f['v_err_rpm']}")
        if log.log_id == 60:
            return f"{f['CCR2']}, {f['CCR3']}, {f['CCR4']}"
        if log.log_id == 70:
            return f"{f['CCR2']}, {f['CCR3']}, {f['CCR4']}, {f['I_a']}, {f['I_b']}, {f['I_c']}"
        if log.log_id == 90:
            return f"{f['Ia_raw']}, {f['Ib_raw']}, {f['Ic_raw']}"
        if log.log_id == 100:
            # firmware 用 %f 格式, 这里也转 float
            return (f"position: {f['pos_ref']/1024.0:.6f}, {f['pos_fb']/1024.0:.6f}, "
                    f"{f['pos_err']/1024.0:.6f}, {f['mech_offset_out']}")
        return None
