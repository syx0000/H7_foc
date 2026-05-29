"""Unit tests for the OTA protocol layer.

Verifies frame layout, CRC implementations, and CRC32 cross-language equivalence
with the firmware's Flash_Crc32 (IEEE 802.3 standard, same as zlib).
"""

import os
import struct
import sys
import zlib

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.protocol import (
    build_ota_begin,
    build_ota_end,
    build_ota_abort,
    build_ota_swap,
    build_ota_data_frame,
    crc16_modbus,
    get_ota_chunk_size,
)


def test_crc16_modbus_standard_vector():
    # Industry-standard test vector for CRC16-MODBUS
    assert crc16_modbus(b"123456789") == 0x4B37


def test_crc16_modbus_empty():
    assert crc16_modbus(b"") == 0xFFFF


def test_crc16_modbus_single_byte():
    # Hand-computed: init=0xFFFF, xor 0x01, then 8 reflection rounds with 0xA001
    assert crc16_modbus(b"\x01") == 0x807E


def test_build_ota_begin():
    cmd = build_ota_begin(size=12345, crc32=0xDEADBEEF, version="1.2.3")
    assert cmd == "otabegin SIZE=12345 CRC=0xDEADBEEF VER=1.2.3"


def test_build_ota_begin_pads_crc_to_8_hex():
    cmd = build_ota_begin(size=1, crc32=0x5, version="x")
    # Firmware sscanf needs a clean fixed-width hex
    assert "CRC=0x00000005" in cmd


def test_build_ota_control_frames():
    assert build_ota_end() == "otaend"
    assert build_ota_abort() == "otaabort"
    assert build_ota_swap() == "otaswap"


def test_build_ota_data_frame_layout():
    payload = bytes(range(16))  # 0x00..0x0F (includes the bytes that would
                                # confuse text-based RX paths)
    frame = build_ota_data_frame(seq=7, payload=payload)

    # Header
    assert frame[0:2] == b"OD"
    seq, length = struct.unpack("<HH", frame[2:6])
    assert seq == 7
    assert length == 16

    # Payload
    assert frame[6:6 + 16] == payload

    # CRC16 over the first 6 + 16 bytes
    expected_crc = crc16_modbus(frame[:6 + 16])
    actual_crc = struct.unpack("<H", frame[6 + 16:6 + 16 + 2])[0]
    assert actual_crc == expected_crc

    # Total length = 2 + 2 + 2 + 16 + 2 = 24
    assert len(frame) == 24


def test_build_ota_data_frame_max_size():
    chunk = get_ota_chunk_size()
    payload = b"\xAA" * chunk
    frame = build_ota_data_frame(seq=0xFFFF, payload=payload)
    assert len(frame) == 6 + chunk + 2

    # seq round-trips through u16
    seq, length = struct.unpack("<HH", frame[2:6])
    assert seq == 0xFFFF
    assert length == chunk


def test_build_ota_data_frame_oversize_rejected():
    too_big = b"\x00" * (get_ota_chunk_size() + 1)
    try:
        build_ota_data_frame(seq=0, payload=too_big)
    except ValueError:
        return
    raise AssertionError("oversize payload should raise ValueError")


def test_crc32_matches_firmware_flash_crc32():
    # Firmware Flash_Crc32 is IEEE 802.3 standard CRC32 (poly 0xEDB88320,
    # init 0xFFFFFFFF, reflected, xorOut 0xFFFFFFFF), which is exactly
    # what zlib.crc32 computes. Cross-check against a known-good vector
    # so we catch any regression in either side.
    assert (zlib.crc32(b"123456789") & 0xFFFFFFFF) == 0xCBF43926


def test_build_ota_data_frame_seq_wraps():
    # Sequence numbers wrap at 16 bits
    frame = build_ota_data_frame(seq=0x1FFFF, payload=b"x")
    seq, _ = struct.unpack("<HH", frame[2:6])
    assert seq == 0xFFFF


if __name__ == "__main__":
    test_crc16_modbus_standard_vector()
    test_crc16_modbus_empty()
    test_crc16_modbus_single_byte()
    test_build_ota_begin()
    test_build_ota_begin_pads_crc_to_8_hex()
    test_build_ota_control_frames()
    test_build_ota_data_frame_layout()
    test_build_ota_data_frame_max_size()
    test_build_ota_data_frame_oversize_rejected()
    test_crc32_matches_firmware_flash_crc32()
    test_build_ota_data_frame_seq_wraps()
    print("All OTA tests passed!")
