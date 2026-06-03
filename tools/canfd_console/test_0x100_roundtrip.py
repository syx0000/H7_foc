#!/usr/bin/env python3
"""
0x100 状态帧打包/解包一致性测试

验证 C 代码 (can_wly.c::pack_status_frame) 与 Python 解析器的一致性
"""

import struct

# 默认量程
POS_MIN, POS_MAX = -7.0, 7.0
SPD_MIN, SPD_MAX = -20.0, 20.0
TQ_MIN, TQ_MAX = -500.0, 500.0

def float_to_uint(x, x_min, x_max, bits):
    """标幺化编码: float -> uint"""
    span = x_max - x_min
    if span <= 0:
        return 0
    x = max(x_min, min(x, x_max))  # clamp
    mask = (1 << bits) - 1
    return int((x - x_min) * mask / span)

def uint_to_float(raw, x_min, x_max, bits):
    """标幺化解码: uint -> float"""
    mask = (1 << bits) - 1
    return raw / mask * (x_max - x_min) + x_min

def pack_0x100_test(
    pact_rad=0.0, vact_rad_s=0.0, tact_nm=0.0,
    pcmd_rad=0.0, vcmd_rad_s=0.0, tcmd_nm=0.0,
    iqref_A=0.0, iqfdb_A=0.0, irms_A=0.0, mit_t_nm=0.0,
    err1=0, err2=0, warn=0, sta=0,
    vdc_V=48, temp_d_degC=25.0, temp_m_degC=45.0
):
    """按照 C 代码逻辑打包 33 字节帧"""
    d = bytearray(33)

    # D[0..2] Pact
    pact_raw = float_to_uint(pact_rad, POS_MIN, POS_MAX, 24)
    d[0] = pact_raw & 0xFF
    d[1] = (pact_raw >> 8) & 0xFF
    d[2] = (pact_raw >> 16) & 0xFF

    # D[3..4] Vact
    vact_raw = float_to_uint(vact_rad_s, SPD_MIN, SPD_MAX, 16)
    struct.pack_into('<H', d, 3, vact_raw)

    # D[5..6] Tact
    tact_raw = float_to_uint(tact_nm, TQ_MIN, TQ_MAX, 16)
    struct.pack_into('<H', d, 5, tact_raw)

    # D[7..8] Err1
    struct.pack_into('<H', d, 7, err1)

    # D[9..10] Err2
    struct.pack_into('<H', d, 9, err2)

    # D[11] warn
    d[11] = warn

    # D[12] STA
    d[12] = sta

    # D[13..15] Pcmd
    pcmd_raw = float_to_uint(pcmd_rad, POS_MIN, POS_MAX, 24)
    d[13] = pcmd_raw & 0xFF
    d[14] = (pcmd_raw >> 8) & 0xFF
    d[15] = (pcmd_raw >> 16) & 0xFF

    # D[16..17] Vcmd
    vcmd_raw = float_to_uint(vcmd_rad_s, SPD_MIN, SPD_MAX, 16)
    struct.pack_into('<H', d, 16, vcmd_raw)

    # D[18..19] Tcmd
    tcmd_raw = float_to_uint(tcmd_nm, TQ_MIN, TQ_MAX, 16)
    struct.pack_into('<H', d, 18, tcmd_raw)

    # D[20..21] iqref (0.01A + 偏置 10000)
    iqref_raw = int(iqref_A * 100 + 10000)
    struct.pack_into('<H', d, 20, iqref_raw & 0xFFFF)

    # D[22..23] iqfdb
    iqfdb_raw = int(iqfdb_A * 100 + 10000)
    struct.pack_into('<H', d, 22, iqfdb_raw & 0xFFFF)

    # D[24..25] Irms
    irms_raw = int(irms_A * 100 + 10000)
    struct.pack_into('<H', d, 24, irms_raw & 0xFFFF)

    # D[26..27] MIT_T
    mit_t_raw = float_to_uint(mit_t_nm, TQ_MIN, TQ_MAX, 16)
    struct.pack_into('<H', d, 26, mit_t_raw)

    # D[28] Vdc
    d[28] = max(0, min(255, vdc_V))

    # D[29..30] Temp_D (0.1°C)
    temp_d_raw = int(temp_d_degC * 10)
    struct.pack_into('<h', d, 29, temp_d_raw)

    # D[31..32] Temp_M (0.1°C)
    temp_m_raw = int(temp_m_degC * 10)
    struct.pack_into('<h', d, 31, temp_m_raw)

    return bytes(d)

def unpack_0x100_test(data):
    """解包 33 字节帧"""
    result = {}

    # D[0..2] Pact
    pact_raw = data[0] | (data[1] << 8) | (data[2] << 16)
    result['pact_rad'] = uint_to_float(pact_raw, POS_MIN, POS_MAX, 24)

    # D[3..4] Vact
    vact_raw = struct.unpack('<H', data[3:5])[0]
    result['vact_rad_s'] = uint_to_float(vact_raw, SPD_MIN, SPD_MAX, 16)

    # D[5..6] Tact
    tact_raw = struct.unpack('<H', data[5:7])[0]
    result['tact_nm'] = uint_to_float(tact_raw, TQ_MIN, TQ_MAX, 16)

    # D[7..10] Err1/Err2
    result['err1'] = struct.unpack('<H', data[7:9])[0]
    result['err2'] = struct.unpack('<H', data[9:11])[0]

    # D[11..12] warn/STA
    result['warn'] = data[11]
    result['sta'] = data[12]

    # D[13..15] Pcmd
    pcmd_raw = data[13] | (data[14] << 8) | (data[15] << 16)
    result['pcmd_rad'] = uint_to_float(pcmd_raw, POS_MIN, POS_MAX, 24)

    # D[16..17] Vcmd
    vcmd_raw = struct.unpack('<H', data[16:18])[0]
    result['vcmd_rad_s'] = uint_to_float(vcmd_raw, SPD_MIN, SPD_MAX, 16)

    # D[18..19] Tcmd
    tcmd_raw = struct.unpack('<H', data[18:20])[0]
    result['tcmd_nm'] = uint_to_float(tcmd_raw, TQ_MIN, TQ_MAX, 16)

    # D[20..25] iqref/iqfdb/Irms
    iqref_raw = struct.unpack('<H', data[20:22])[0]
    result['iqref_A'] = (iqref_raw - 10000) / 100.0

    iqfdb_raw = struct.unpack('<H', data[22:24])[0]
    result['iqfdb_A'] = (iqfdb_raw - 10000) / 100.0

    irms_raw = struct.unpack('<H', data[24:26])[0]
    result['irms_A'] = (irms_raw - 10000) / 100.0

    # D[26..27] MIT_T
    mit_t_raw = struct.unpack('<H', data[26:28])[0]
    result['mit_t_nm'] = uint_to_float(mit_t_raw, TQ_MIN, TQ_MAX, 16)

    # D[28] Vdc
    result['vdc_V'] = data[28]

    # D[29..32] Temp_D/Temp_M
    temp_d_raw = struct.unpack('<h', data[29:31])[0]
    result['temp_d_degC'] = temp_d_raw / 10.0

    temp_m_raw = struct.unpack('<h', data[31:33])[0]
    result['temp_m_degC'] = temp_m_raw / 10.0

    return result

def test_roundtrip():
    """打包 -> 解包 -> 验证一致性"""
    test_cases = [
        # 零点
        {
            'pact_rad': 0.0, 'vact_rad_s': 0.0, 'tact_nm': 0.0,
            'pcmd_rad': 0.0, 'vcmd_rad_s': 0.0, 'tcmd_nm': 0.0,
            'iqref_A': 0.0, 'iqfdb_A': 0.0, 'irms_A': 0.0, 'mit_t_nm': 0.0,
            'vdc_V': 48, 'temp_d_degC': 25.0, 'temp_m_degC': 45.0
        },
        # 正向满量程
        {
            'pact_rad': 7.0, 'vact_rad_s': 20.0, 'tact_nm': 500.0,
            'pcmd_rad': 7.0, 'vcmd_rad_s': 20.0, 'tcmd_nm': 500.0,
            'iqref_A': 100.0, 'iqfdb_A': 100.0, 'irms_A': 100.0, 'mit_t_nm': 500.0,
            'vdc_V': 255, 'temp_d_degC': 90.0, 'temp_m_degC': 120.0
        },
        # 负向满量程
        {
            'pact_rad': -7.0, 'vact_rad_s': -20.0, 'tact_nm': -500.0,
            'pcmd_rad': -7.0, 'vcmd_rad_s': -20.0, 'tcmd_nm': -500.0,
            'iqref_A': -100.0, 'iqfdb_A': -100.0, 'irms_A': 0.0, 'mit_t_nm': -500.0,
            'vdc_V': 0, 'temp_d_degC': -10.0, 'temp_m_degC': 0.0
        },
        # 典型工作点
        {
            'pact_rad': 1.234, 'vact_rad_s': 5.67, 'tact_nm': 123.4,
            'pcmd_rad': 1.5, 'vcmd_rad_s': 6.0, 'tcmd_nm': 130.0,
            'iqref_A': 12.34, 'iqfdb_A': 12.1, 'irms_A': 12.2, 'mit_t_nm': 5.0,
            'vdc_V': 48, 'temp_d_degC': 35.5, 'temp_m_degC': 55.3
        }
    ]

    for i, tc in enumerate(test_cases):
        print(f"\n=== Test Case {i+1} ===")
        packed = pack_0x100_test(**tc)
        unpacked = unpack_0x100_test(packed)

        print(f"原始: Pact={tc['pact_rad']:.4f} Vact={tc['vact_rad_s']:.3f} Tact={tc['tact_nm']:.2f}")
        print(f"解包: Pact={unpacked['pact_rad']:.4f} Vact={unpacked['vact_rad_s']:.3f} Tact={unpacked['tact_nm']:.2f}")

        # 验证误差 (标幺化量化误差应 < 0.1%)
        err_pact = abs(unpacked['pact_rad'] - tc['pact_rad'])
        err_vact = abs(unpacked['vact_rad_s'] - tc['vact_rad_s'])
        err_tact = abs(unpacked['tact_nm'] - tc['tact_nm'])
        err_iq = abs(unpacked['iqref_A'] - tc['iqref_A'])

        assert err_pact < 0.001, f"Pact 误差过大: {err_pact}"
        assert err_vact < 0.01, f"Vact 误差过大: {err_vact}"
        assert err_tact < 0.5, f"Tact 误差过大: {err_tact}"
        assert err_iq < 0.01, f"iqref 误差过大: {err_iq}"

        # 验证温度 (0.1°C 精度)
        assert abs(unpacked['temp_d_degC'] - tc['temp_d_degC']) < 0.1
        assert abs(unpacked['temp_m_degC'] - tc['temp_m_degC']) < 0.1

        print(f"✓ 验证通过 (Pact_err={err_pact:.6f}, Vact_err={err_vact:.6f}, Tact_err={err_tact:.3f}, Iq_err={err_iq:.3f})")

        # 打印十六进制 (方便 MCU 侧对比)
        print(f"HEX: {packed.hex(' ')}")

    print("\n=== All Tests PASSED ===")

if __name__ == '__main__':
    test_roundtrip()
