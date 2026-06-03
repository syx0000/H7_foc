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
            import traceback
            traceback.print_exc()
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
            cmd = cmd.strip()

            # 特殊处理: phase_comp 的 4 行命令 -> 单个 CAN 帧
            if '\r\n' in cmd or '\n' in cmd:
                lines = [l.strip() for l in cmd.replace('\r\n', '\n').split('\n') if l.strip()]
                # 检查是否是 phase_comp 4 合 1
                if len(lines) == 4 and all(
                    lines[i].startswith(prefix) for i, prefix in enumerate(['offsetpos', 'offsetneg', 'comppos', 'compneg'])
                ):
                    # 解析 4 个值 - 精确匹配命令名后的数字
                    vals = []
                    patterns = [r'offsetpos\s*(-?\d+)$', r'offsetneg\s*(-?\d+)$',
                               r'comppos\s*(-?\d+)$', r'compneg\s*(-?\d+)$']
                    for line, pattern in zip(lines, patterns):
                        m = re.match(pattern, line)
                        if m:
                            vals.append(int(m.group(1)))
                        else:
                            vals.append(0)
                    frame = proto.pack_phase_comp_set(*vals)
                    self._bus.send(proto.ID_CMD, frame)
                    return
                # 其他多行命令：逐个发送
                for line in lines:
                    frame = self._translate_cmd(line)
                    if frame:
                        self._bus.send(proto.ID_CMD, frame)
                return

            frame = self._translate_cmd(cmd.rstrip('\r\n'))
            if frame is None:
                return  # _translate_cmd 已经发出了错误信号或命令不支持
            self._bus.send(proto.ID_CMD, frame)
        except Exception as e:
            self.sig_error.emit(f"CAN send error: {e}")
        finally:
            self._mutex.unlock()

    def send_wly_speed(self, speed_rpm: float, node_id: int = 1):
        """发送万里扬速度指令 (0x200).

        使用归一化映射: v_raw = (v_rad_s - v_min) / (v_max - v_min) * 65535
        默认量程: ±20 rad/s (±191 rpm 输出端)
        """
        self._mutex.lock()
        try:
            if not self._bus:
                return
            # rpm -> rad/s (输出端)
            v_rad_s = speed_rpm * (2.0 * 3.14159265 / 60.0)
            # 归一化映射到 uint16
            v_min, v_max = -20.0, 20.0
            if v_rad_s < v_min:
                v_rad_s = v_min
            elif v_rad_s > v_max:
                v_rad_s = v_max
            v_raw = int((v_rad_s - v_min) / (v_max - v_min) * 65535.0)
            v_raw = max(0, min(65535, v_raw))
            frame = bytes([v_raw & 0xFF, (v_raw >> 8) & 0xFF, node_id])
            self._bus.send(0x200, frame, brs=True)
        except Exception as e:
            self.sig_error.emit(f"CAN send_wly_speed error: {e}")
        finally:
            self._mutex.unlock()

    def send_wly_position(self, pos_deg: float, speed_rpm: float, node_id: int = 1):
        """发送万里扬位置指令 (0x400).

        使用归一化映射: p_raw = (p_rad - p_min) / (p_max - p_min) * 16777215
        默认量程: ±7 rad (±401° 输出端), 速度 ±20 rad/s
        """
        self._mutex.lock()
        try:
            if not self._bus:
                return
            # deg -> rad (输出端)
            p_rad = pos_deg * (3.14159265 / 180.0)
            # 归一化映射到 uint24
            p_min, p_max = -7.0, 7.0
            if p_rad < p_min:
                p_rad = p_min
            elif p_rad > p_max:
                p_rad = p_max
            p_raw = int((p_rad - p_min) / (p_max - p_min) * 16777215.0)
            p_raw = max(0, min(16777215, p_raw))

            # 速度 rpm -> rad/s
            v_rad_s = speed_rpm * (2.0 * 3.14159265 / 60.0)
            v_min, v_max = -20.0, 20.0
            if v_rad_s < v_min:
                v_rad_s = v_min
            elif v_rad_s > v_max:
                v_rad_s = v_max
            v_raw = int((v_rad_s - v_min) / (v_max - v_min) * 65535.0)
            v_raw = max(0, min(65535, v_raw))

            frame = bytes([
                p_raw & 0xFF,
                (p_raw >> 8) & 0xFF,
                (p_raw >> 16) & 0xFF,
                v_raw & 0xFF,
                (v_raw >> 8) & 0xFF,
                node_id
            ])
            self._bus.send(0x400, frame, brs=True)
        except Exception as e:
            self.sig_error.emit(f"CAN send_wly_position error: {e}")
        finally:
            self._mutex.unlock()

    def send_wly_torque(self, torque_nm: float, node_id: int = 1):
        """发送万里扬转矩指令 (0x300).

        使用归一化映射: t_raw = (t_nm - t_min) / (t_max - t_min) * 65535
        默认量程: ±500 N·m
        """
        self._mutex.lock()
        try:
            if not self._bus:
                return
            # 归一化映射到 uint16
            t_min, t_max = -500.0, 500.0
            if torque_nm < t_min:
                torque_nm = t_min
            elif torque_nm > t_max:
                torque_nm = t_max
            t_raw = int((torque_nm - t_min) / (t_max - t_min) * 65535.0)
            t_raw = max(0, min(65535, t_raw))
            frame = bytes([t_raw & 0xFF, (t_raw >> 8) & 0xFF, node_id])
            self._bus.send(0x300, frame, brs=True)
        except Exception as e:
            self.sig_error.emit(f"CAN send_wly_torque error: {e}")
        finally:
            self._mutex.unlock()

    def send_wly_enable(self, enable: bool, node_id: int = 1):
        """发送万里扬使能/失能控制帧 (0x700+ID).

        格式: D[0:6]=0xFF, D[7]=0xFA(使能)/0xFB(失能)
        """
        self._mutex.lock()
        try:
            if not self._bus:
                return
            cmd_byte = 0xFA if enable else 0xFB
            frame = bytes([0xFF] * 7 + [cmd_byte])
            can_id = 0x700 + node_id
            self._bus.send(can_id, frame, brs=True)
        except Exception as e:
            self.sig_error.emit(f"CAN send_wly_enable error: {e}")
        finally:
            self._mutex.unlock()

    def send_wly_clr_err(self, node_id: int = 1):
        """发送万里扬清错控制帧 (0x700+ID, D[7]=0xFD)."""
        self._mutex.lock()
        try:
            if not self._bus:
                return
            frame = bytes([0xFF] * 7 + [0xFD])
            can_id = 0x700 + node_id
            self._bus.send(can_id, frame, brs=True)
        except Exception as e:
            self.sig_error.emit(f"CAN send_wly_clr_err error: {e}")
        finally:
            self._mutex.unlock()

    # ---------- 命令文本 -> 0x7E0 二进制 ----------
    def _translate_cmd(self, cmd: str) -> bytes | None:
        """串口文本命令 -> CAN 二进制帧. 返回 None 表示不支持."""
        # logid 160/161/163 是串口兼容的"立即动作"命令, CAN 模式下转为对应的专用命令
        # 这样可以触发 MCU 端的 h_flash_write/h_flash_erase/h_fault_clr (会发 TEXT 反馈)
        if cmd == 'logid160':
            return proto.pack_flash_write()
        if cmd == 'logid161':
            return proto.pack_flash_erase()
        if cmd == 'logid162':
            return proto.pack_flash_compare()
        if cmd == 'logid163':
            return proto.pack_fault_clear()
        # logid162 (Flash 对比) 和 logid165 (查故障) 无对应 CAN 专用命令,
        # 直接走 LOGID_SET, MCU 主循环 dbg_log_print 走串口 printf (CAN 模式下看不到)
        # 这是预期行为, 详细信息建议用串口

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

        # offsetpos/offsetneg/comppos/compneg - 单独命令时也支持
        m = re.match(r'offsetpos\s*(-?\d+)$', cmd)
        if m:
            return proto.pack_phase_comp_set(int(m.group(1)), 0, 0, 0)
        m = re.match(r'offsetneg\s*(-?\d+)$', cmd)
        if m:
            return proto.pack_phase_comp_set(0, int(m.group(1)), 0, 0)
        m = re.match(r'comppos\s*(-?\d+)$', cmd)
        if m:
            return proto.pack_phase_comp_set(0, 0, int(m.group(1)), 0)
        m = re.match(r'compneg\s*(-?\d+)$', cmd)
        if m:
            return proto.pack_phase_comp_set(0, 0, 0, int(m.group(1)))

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

        # bwtest<N>: 带宽测试/辨识 - 现在通过 CAN 支持
        m = re.match(r'bwtest\s*(\d+)$', cmd)
        if m:
            return proto.pack_bwtest(int(m.group(1)))

        # Cali: 电角度标定 - 现在通过 CAN 支持
        if cmd == 'Cali':
            return proto.pack_cali()

        # getparams: 查询参数
        if cmd == 'getparams':
            return proto.pack_get_params()

        # OTA: 固件升级 - CAN 调试协议暂不支持
        if cmd.startswith('ota'):
            self.sig_error.emit("OTA requires serial connection")
            return None

        # logid 160/161/163 经常被 GUI 用作"立即动作" (写/擦/清错)
        # 这些已经被 logid<N> 路径处理了 (MCU 端 dbg_log_print 的 case 160/161/163)
        # 但走 CAN 时 dbgLogFlag 设了, 主循环 dbg_log_print 也会执行, OK

        # 未识别的命令
        self.sig_error.emit(f"CAN: unsupported cmd {cmd!r}")
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
        if cid == proto.ID_TEXT:
            # 0x7E6 文本帧：直接返回字符串
            try:
                text = data.decode('utf-8', errors='replace').rstrip('\x00\r\n')
                return text
            except Exception:
                return None
        if cid == proto.ID_RESP:
            resp = proto.parse_resp(data)
            if resp.cmd == proto.CMD.VERSION and resp.ok:
                v = proto.parse_version_payload(resp.payload)
                return f"FW SW={v.soft} HW={v.hard} build {v.build}"
            if resp.cmd == proto.CMD.GET_PARAMS and resp.ok:
                # 转换为 GUI 期望的多行文本格式 (兼容串口)
                p = proto.parse_params_payload(resp.payload)
                lines = [
                    "PARAMS_BEGIN",
                    f"CurKp={p.cur_kp} CurKi={p.cur_ki} CurKd={p.cur_kd}",
                    f"SpdKp={p.spd_kp} SpdKi={p.spd_ki} SpdKd={p.spd_kd}",
                    f"PosKp={p.pos_kp} PosKi={p.pos_ki} PosKd={p.pos_kd}",
                    f"OffPos={p.off_pos} OffNeg={p.off_neg} CompPos={p.comp_pos} CompNeg={p.comp_neg}",
                    "PARAMS_END"
                ]
                # 逐行 emit (GUI 的 process_line 期望收到多行)
                for line in lines:
                    self.sig_line_received.emit(line)
                return None  # 已经 emit 了，不要再返回
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
