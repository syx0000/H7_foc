"""
test_protocol.py — 协议层离线测试 (不需要硬件)

运行: cd tools/canfd_console && python -m pytest test_protocol.py -v
或:   cd tools/canfd_console && python test_protocol.py
"""
import struct
import unittest

import can_debug_protocol as proto


class TestPack(unittest.TestCase):
    def test_pack_ping(self):
        self.assertEqual(proto.pack_ping(), b"\x00")

    def test_pack_version(self):
        self.assertEqual(proto.pack_version(), b"\x01")

    def test_pack_reset(self):
        self.assertEqual(proto.pack_reset(), b"\x02")


class TestParseResp(unittest.TestCase):
    def test_ok_resp(self):
        # CMD=0x00 PING ok: [0x00][0x00][0x01][0x00] (proto_ver=1)
        r = proto.parse_resp(b"\x00\x00\x01\x00")
        self.assertEqual(r.cmd, proto.CMD.PING)
        self.assertTrue(r.ok)
        self.assertIsNone(r.err)
        self.assertEqual(r.payload, b"\x01\x00")

    def test_err_resp(self):
        # 错误: [0xFF][CMD=0x99][ERR=0x01_UNKNOWN]
        r = proto.parse_resp(b"\xFF\x99\x01")
        self.assertEqual(r.cmd, 0x99)
        self.assertFalse(r.ok)
        self.assertEqual(r.err, proto.ERR.UNKNOWN_CMD)
        self.assertEqual(r.payload, b"")

    def test_err_resp_short(self):
        with self.assertRaises(ValueError):
            proto.parse_resp(b"\xFF\x99")  # 缺 ERR 字节

    def test_empty(self):
        with self.assertRaises(ValueError):
            proto.parse_resp(b"")


class TestVersionPayload(unittest.TestCase):
    def test_normal(self):
        # 29B: soft=10 + hw=8 + build=11 (32B FIFO 约束, 见 CAN_DEBUG_DESIGN.md §3.2)
        soft  = b"20260528.1"
        hard  = b"20260528"
        build = b"Jun  1 2026"
        v = proto.parse_version_payload(soft + hard + build)
        self.assertEqual(v.soft, "20260528.1")
        self.assertEqual(v.hard, "20260528")
        self.assertEqual(v.build, "Jun  1 2026")

    def test_too_short(self):
        with self.assertRaises(ValueError):
            proto.parse_version_payload(b"\x00" * 10)


class TestPingPayload(unittest.TestCase):
    def test_normal(self):
        info = proto.parse_ping_payload(b"\x01\x00")
        self.assertEqual(info.proto_ver, 1)


class TestPhase3Pack(unittest.TestCase):
    """Phase 3 命令打包测试"""

    def test_logid_set(self):
        # CMD=0x10, log_id=50 (LE u16)
        self.assertEqual(proto.pack_logid_set(50), b"\x10\x32\x00")

    def test_logfreq_set(self):
        self.assertEqual(proto.pack_logfreq_set(100), b"\x11\x64\x00")

    def test_pid_set(self):
        # CUR_PID kp=45, ki=4, kd=0 -> 1+4+4+4=13B
        frame = proto.pack_pid_set(proto.CMD.CUR_PID_SET, 45, 4, 0)
        self.assertEqual(len(frame), 13)
        self.assertEqual(frame[0], 0x20)
        # kp=45 LE u32
        self.assertEqual(frame[1:5], b"\x2D\x00\x00\x00")
        # ki=4
        self.assertEqual(frame[5:9], b"\x04\x00\x00\x00")
        # kd=0
        self.assertEqual(frame[9:13], b"\x00\x00\x00\x00")

    def test_enable(self):
        self.assertEqual(proto.pack_enable(True), b"\x50\x01")
        self.assertEqual(proto.pack_enable(False), b"\x50\x00")

    def test_phase_comp_set(self):
        # CMD=0x52, off_pos=100, off_neg=-50, comp_pos=20, comp_neg=-30
        frame = proto.pack_phase_comp_set(100, -50, 20, -30)
        self.assertEqual(len(frame), 9)  # 1+2*4=9
        self.assertEqual(frame[0], 0x52)
        # off_pos=100 (i16 LE) -> 0x64 0x00
        self.assertEqual(frame[1:3], b"\x64\x00")
        # off_neg=-50 (i16 LE) -> 0xCE 0xFF
        self.assertEqual(frame[3:5], b"\xCE\xFF")

    def test_simple_cmds(self):
        self.assertEqual(proto.pack_flash_write(), b"\x40")
        self.assertEqual(proto.pack_flash_erase(), b"\x41")
        self.assertEqual(proto.pack_fault_clear(), b"\x43")
        self.assertEqual(proto.pack_phase_comp_save(), b"\x53")
        self.assertEqual(proto.pack_canrxdbg(True), b"\x61\x01")
        self.assertEqual(proto.pack_canrxdbg(False), b"\x61\x00")


class TestPhase4Log(unittest.TestCase):
    """Phase 4 周期日志 / 异步事件解析测试"""

    def test_log_50_speed(self):
        # LOG_ID=50, seq=7, ts=0x1234, payload: 5×i32 LE
        # v_ref=100rpm, v_ref_filt=99rpm, v_fb_motor=2500rpm, v_fb_load=1000(=100.0rpm 输出端 0.1rpm/LSB), v_err=1rpm
        import struct
        hdr = struct.pack('<BBH', 50, 7, 0x1234)
        payload = struct.pack('<iiiii', 100, 99, 2500, 1000, 1)
        log = proto.parse_log(hdr + payload)
        self.assertEqual(log.log_id, 50)
        self.assertEqual(log.seq, 7)
        self.assertEqual(log.ts_ms, 0x1234)
        self.assertEqual(log.fields['v_ref_rpm'], 100)
        self.assertEqual(log.fields['v_fb_motor_rpm'], 2500)
        self.assertEqual(log.fields['v_fb_load_0p1rpm'], 1000)

    def test_log_40_current(self):
        import struct
        hdr = struct.pack('<BBH', 40, 0, 0)
        payload = struct.pack('<iiiiiii', 1024, -512, 100, 50, 2048, 0, 1024)
        log = proto.parse_log(hdr + payload)
        self.assertEqual(log.log_id, 40)
        self.assertEqual(log.fields['I_q'], 1024)
        self.assertEqual(log.fields['I_d'], -512)
        self.assertEqual(log.fields['I_q_ref'], 2048)

    def test_log_unknown_id(self):
        # 未知 LOG_ID 返回 raw payload, 不报错
        log = proto.parse_log(b"\xAA\x00\x00\x00\x01\x02\x03")
        self.assertEqual(log.log_id, 0xAA)
        self.assertEqual(log.fields, {'raw': '010203'})

    def test_log_too_short(self):
        with self.assertRaises(ValueError):
            proto.parse_log(b"\x00\x00")  # 头都不够

    def test_event_parse(self):
        evt = proto.parse_event(b"\x30\x03\x00\x01\x02")
        self.assertEqual(evt.event_id, 0x30)  # BWTEST_DONE
        self.assertEqual(evt.payload, b"\x03\x00\x01\x02")


if __name__ == "__main__":
    unittest.main(verbosity=2)
