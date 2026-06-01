"""
cxcanfd_driver.py — 创芯 USBCANFD-2CH 驱动封装

基于 tools/canfd/python(x64)_example_python3.8.8_v2.0_en/cxcanfd_x64_v2.0.py 改写为类接口.
保持与官方 demo 同名 ctypes 结构体, 方便对照手册排错.

依赖: ControlCANFD.dll (与本文件同目录)
平台: Windows-only (Python 3.8+, 64-bit)
"""
from __future__ import annotations

import os
import sys
from ctypes import (
    Structure, Union, byref, c_ubyte, c_uint, c_ulong, c_void_p, c_long, c_ulonglong, c_ushort, windll
)
from typing import Optional

# ===== 常量 (与 demo 一致) =====
VCI_USBCAN2 = 41           # USBCANFD-2CH 设备类型代码
STATUS_OK = 1
INVALID_DEVICE_HANDLE = 0
INVALID_CHANNEL_HANDLE = 0
TYPE_CAN = 0
TYPE_CANFD = 1


# ===== ctypes 结构体 =====
class _ZCAN_CHANNEL_CAN_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint), ("acc_mask", c_uint), ("reserved", c_uint),
                ("filter", c_ubyte), ("timing0", c_ubyte), ("timing1", c_ubyte),
                ("mode", c_ubyte)]


class _ZCAN_CHANNEL_CANFD_INIT_CONFIG(Structure):
    _fields_ = [("acc_code", c_uint), ("acc_mask", c_uint), ("abit_timing", c_uint),
                ("dbit_timing", c_uint), ("brp", c_uint), ("filter", c_ubyte),
                ("mode", c_ubyte), ("pad", c_ushort), ("reserved", c_uint)]


class _ZCAN_CHANNEL_INIT_CONFIG(Union):
    _fields_ = [("can", _ZCAN_CHANNEL_CAN_INIT_CONFIG),
                ("canfd", _ZCAN_CHANNEL_CANFD_INIT_CONFIG)]


class ZCAN_CHANNEL_INIT_CONFIG(Structure):
    _fields_ = [("can_type", c_uint), ("config", _ZCAN_CHANNEL_INIT_CONFIG)]


class ZCAN_CANFD_FRAME(Structure):
    _fields_ = [("can_id", c_uint, 29), ("err", c_uint, 1), ("rtr", c_uint, 1),
                ("eff", c_uint, 1), ("len", c_ubyte),
                ("brs", c_ubyte, 1), ("esi", c_ubyte, 1), ("__res", c_ubyte, 6),
                ("__res0", c_ubyte), ("__res1", c_ubyte),
                ("data", c_ubyte * 64)]


class ZCAN_TransmitFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("transmit_type", c_uint)]


class ZCAN_ReceiveFD_Data(Structure):
    _fields_ = [("frame", ZCAN_CANFD_FRAME), ("timestamp", c_ulonglong)]


# ===== 长度 -> DLC 映射 (CAN-FD 离散值 0/1/.../8/12/16/20/24/32/48/64) =====
_FD_LEN_TABLE = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


def _round_up_fd_len(n: int) -> int:
    for v in _FD_LEN_TABLE:
        if v >= n:
            return v
    return 64


# ===== 驱动类 =====
class CXCanFD:
    """创芯 USBCANFD 驱动. 用 with 语句保证 close.

    Example:
        with CXCanFD(channel=0, abit=1_000_000, dbit=5_000_000) as bus:
            bus.send(0x7E0, b"\\x00")          # PING
            cid, data, ts = bus.recv(timeout_ms=200)
    """

    def __init__(self, dev_index: int = 0, channel: int = 0,
                 abit: int = 1_000_000, dbit: int = 5_000_000,
                 dll_path: Optional[str] = None):
        if dll_path is None:
            dll_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    "ControlCANFD.dll")
        if not os.path.exists(dll_path):
            raise FileNotFoundError(f"ControlCANFD.dll not found at {dll_path}")
        self.dll = windll.LoadLibrary(dll_path)
        self._setup_argtypes()

        self.dev_index = dev_index
        self.channel = channel
        self.dev = self.dll.ZCAN_OpenDevice(VCI_USBCAN2, dev_index, 0)
        if self.dev == INVALID_DEVICE_HANDLE:
            raise RuntimeError(f"ZCAN_OpenDevice failed (type={VCI_USBCAN2}, idx={dev_index})")

        self._check(self.dll.ZCAN_SetAbitBaud(self.dev, channel, abit), f"SetAbitBaud {abit}")
        self._check(self.dll.ZCAN_SetDbitBaud(self.dev, channel, dbit), f"SetDbitBaud {dbit}")
        self._check(self.dll.ZCAN_SetCANFDStandard(self.dev, channel, 0), "SetCANFDStandard ISO")

        cfg = ZCAN_CHANNEL_INIT_CONFIG()
        cfg.can_type = TYPE_CANFD
        cfg.config.canfd.mode = 0
        self.ch = self.dll.ZCAN_InitCAN(self.dev, channel, byref(cfg))
        if self.ch == INVALID_CHANNEL_HANDLE:
            self.dll.ZCAN_CloseDevice(self.dev)
            raise RuntimeError(f"ZCAN_InitCAN failed (ch={channel})")

        self._check(self.dll.ZCAN_StartCAN(self.ch), "StartCAN")

    def _setup_argtypes(self):
        d = self.dll
        d.ZCAN_OpenDevice.restype = c_void_p
        d.ZCAN_SetAbitBaud.argtypes = (c_void_p, c_ulong, c_ulong)
        d.ZCAN_SetDbitBaud.argtypes = (c_void_p, c_ulong, c_ulong)
        d.ZCAN_SetCANFDStandard.argtypes = (c_void_p, c_ulong, c_ulong)
        d.ZCAN_InitCAN.argtypes = (c_void_p, c_ulong, c_void_p)
        d.ZCAN_InitCAN.restype = c_void_p
        d.ZCAN_StartCAN.argtypes = (c_void_p,)
        d.ZCAN_TransmitFD.argtypes = (c_void_p, c_void_p, c_ulong)
        d.ZCAN_GetReceiveNum.argtypes = (c_void_p, c_ulong)
        d.ZCAN_ReceiveFD.argtypes = (c_void_p, c_void_p, c_ulong, c_long)
        d.ZCAN_ResetCAN.argtypes = (c_void_p,)
        d.ZCAN_CloseDevice.argtypes = (c_void_p,)
        d.ZCAN_ClearFilter.argtypes = (c_void_p,)
        d.ZCAN_AckFilter.argtypes = (c_void_p,)
        d.ZCAN_SetFilterMode.argtypes = (c_void_p, c_ulong)
        d.ZCAN_SetFilterStartID.argtypes = (c_void_p, c_ulong)
        d.ZCAN_SetFilterEndID.argtypes = (c_void_p, c_ulong)

    def _check(self, ret: int, what: str):
        if ret != STATUS_OK:
            raise RuntimeError(f"CXCanFD: {what} failed (ret={ret})")

    def set_filter(self, start_id: int, end_id: int, std: bool = True):
        """限制接收 ID 范围. 默认 std=True (标准帧)."""
        self.dll.ZCAN_ClearFilter(self.ch)
        self.dll.ZCAN_SetFilterMode(self.ch, 0 if std else 1)
        self.dll.ZCAN_SetFilterStartID(self.ch, start_id)
        self.dll.ZCAN_SetFilterEndID(self.ch, end_id)
        self.dll.ZCAN_AckFilter(self.ch)

    def send(self, can_id: int, data: bytes, *, brs: bool = True, eff: bool = False) -> bool:
        """发一帧 CAN-FD. data 长度按 FD DLC 自动 padding."""
        if len(data) > 64:
            raise ValueError("CAN-FD 单帧最大 64B")
        msg = ZCAN_TransmitFD_Data()
        msg.transmit_type = 0
        msg.frame.eff = 1 if eff else 0
        msg.frame.rtr = 0
        msg.frame.brs = 1 if brs else 0
        msg.frame.can_id = can_id
        padded = _round_up_fd_len(len(data))
        msg.frame.len = padded
        for i, b in enumerate(data):
            msg.frame.data[i] = b
        # 剩余字节 ctypes 默认 0, 无需手动清零
        n = self.dll.ZCAN_TransmitFD(self.ch, byref(msg), 1)
        return n == 1

    def recv(self, timeout_ms: int = 100):
        """收一帧. 返回 (can_id, data:bytes, timestamp_us) 或 None."""
        # 先 poll 数量, 再 ReceiveFD
        # ReceiveFD 第 4 参数是单位 ms 的等待时间 (-1=阻塞, 0=立即返回)
        # demo 是 -1 阻塞, 这里允许指定超时
        avail = self.dll.ZCAN_GetReceiveNum(self.ch, TYPE_CANFD)
        if avail <= 0:
            # 等一下再 poll 一次 (粗粒度, 创芯 DLL 没暴露真正的阻塞 recv)
            import time
            t_end = time.monotonic() + timeout_ms / 1000.0
            while avail <= 0 and time.monotonic() < t_end:
                time.sleep(0.001)
                avail = self.dll.ZCAN_GetReceiveNum(self.ch, TYPE_CANFD)
            if avail <= 0:
                return None
        bufs = (ZCAN_ReceiveFD_Data * 1)()
        n = self.dll.ZCAN_ReceiveFD(self.ch, byref(bufs), 1, 0)
        if n <= 0:
            return None
        f = bufs[0].frame
        data = bytes(f.data[:f.len])
        return (int(f.can_id), data, int(bufs[0].timestamp))

    def close(self):
        if getattr(self, "ch", None):
            try:
                self.dll.ZCAN_ResetCAN(self.ch)
            except Exception:
                pass
            self.ch = None
        if getattr(self, "dev", None):
            try:
                self.dll.ZCAN_CloseDevice(self.dev)
            except Exception:
                pass
            self.dev = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()


if __name__ == "__main__":
    # 自检: 仅打开设备
    print("CXCanFD self-test: opening device...")
    with CXCanFD() as bus:
        print(f"  Device opened. ch={bus.channel}")
        print("  OK")
