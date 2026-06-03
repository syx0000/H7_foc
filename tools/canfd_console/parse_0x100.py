#!/usr/bin/env python3
"""
0x100+ID 状态帧解析工具

用法:
  python parse_0x100.py --hex "12 34 56 AB CD ... (33字节)"
  python parse_0x100.py --file candump.log
"""

import struct
import sys
import argparse

# 默认量程 (与 can_wly.c 中 g_can_wly_lim 对齐)
POS_MIN = -7.0
POS_MAX = 7.0
SPD_MIN = -20.0
SPD_MAX = 20.0
TQ_MIN = -500.0
TQ_MAX = 500.0

def uint_to_float(raw, x_min, x_max, bits):
    """标幺化解码: raw (uint) -> float"""
    mask = (1 << bits) - 1
    return raw / mask * (x_max - x_min) + x_min

def parse_0x100(data):
    """解析 33 字节 0x100 状态帧"""
    if len(data) < 33:
        print(f"Error: 帧长不足, 需要 33B, 实际 {len(data)}B")
        return None

    result = {}

    # D[0..2] Pact (uint24 LE)
    pact_raw = data[0] | (data[1] << 8) | (data[2] << 16)
    result['Pact_rad'] = uint_to_float(pact_raw, POS_MIN, POS_MAX, 24)

    # D[3..4] Vact (uint16 LE)
    vact_raw = struct.unpack('<H', data[3:5])[0]
    result['Vact_rad_s'] = uint_to_float(vact_raw, SPD_MIN, SPD_MAX, 16)

    # D[5..6] Tact (uint16 LE)
    tact_raw = struct.unpack('<H', data[5:7])[0]
    result['Tact_Nm'] = uint_to_float(tact_raw, TQ_MIN, TQ_MAX, 16)

    # D[7..8] Err1 (uint16 LE)
    result['Err1'] = struct.unpack('<H', data[7:9])[0]

    # D[9..10] Err2 (uint16 LE)
    result['Err2'] = struct.unpack('<H', data[9:11])[0]

    # D[11] warn (uint8)
    warn = data[11]
    result['warn'] = warn
    result['warn_mos_overheat'] = bool(warn & 0x01)
    result['warn_motor_overheat'] = bool(warn & 0x02)

    # D[12] STA (uint8)
    sta = data[12]
    result['STA'] = sta
    result['sta_enabled'] = bool(sta & 0x01)
    result['sta_fault'] = bool(sta & 0x02)
    result['sta_warn'] = bool(sta & 0x04)
    result['sta_arrived'] = bool(sta & 0x08)

    # D[13..15] Pcmd (uint24 LE)
    pcmd_raw = data[13] | (data[14] << 8) | (data[15] << 16)
    result['Pcmd_rad'] = uint_to_float(pcmd_raw, POS_MIN, POS_MAX, 24)

    # D[16..17] Vcmd (uint16 LE)
    vcmd_raw = struct.unpack('<H', data[16:18])[0]
    result['Vcmd_rad_s'] = uint_to_float(vcmd_raw, SPD_MIN, SPD_MAX, 16)

    # D[18..19] Tcmd (uint16 LE)
    tcmd_raw = struct.unpack('<H', data[18:20])[0]
    result['Tcmd_Nm'] = uint_to_float(tcmd_raw, TQ_MIN, TQ_MAX, 16)

    # D[20..21] iqref (uint16 LE, 0.01A + 偏置 10000)
    iqref_raw = struct.unpack('<H', data[20:22])[0]
    result['iqref_A'] = (iqref_raw - 10000) / 100.0

    # D[22..23] iqfdb (uint16 LE, 0.01A + 偏置 10000)
    iqfdb_raw = struct.unpack('<H', data[22:24])[0]
    result['iqfdb_A'] = (iqfdb_raw - 10000) / 100.0

    # D[24..25] Irms (uint16 LE, 0.01A + 偏置 10000)
    irms_raw = struct.unpack('<H', data[24:26])[0]
    result['Irms_A'] = (irms_raw - 10000) / 100.0

    # D[26..27] MIT_T (uint16 LE)
    mit_t_raw = struct.unpack('<H', data[26:28])[0]
    result['MIT_T_Nm'] = uint_to_float(mit_t_raw, TQ_MIN, TQ_MAX, 16)

    # D[28] Vdc (uint8)
    result['Vdc_V'] = data[28]

    # D[29..30] Temp_D (int16 LE, 0.1°C)
    temp_d_raw = struct.unpack('<h', data[29:31])[0]
    result['Temp_D_degC'] = temp_d_raw / 10.0

    # D[31..32] Temp_M (int16 LE, 0.1°C)
    temp_m_raw = struct.unpack('<h', data[31:33])[0]
    result['Temp_M_degC'] = temp_m_raw / 10.0

    return result

def print_result(result):
    """格式化打印解析结果"""
    if result is None:
        return

    print("\n=== 反馈通道 ===")
    print(f"  位置: {result['Pact_rad']:+.4f} rad ({result['Pact_rad']*180/3.14159:+.2f}°)")
    print(f"  速度: {result['Vact_rad_s']:+.3f} rad/s ({result['Vact_rad_s']*60/6.28318:+.1f} rpm)")
    print(f"  转矩: {result['Tact_Nm']:+.2f} N·m")

    print("\n=== 指令通道 ===")
    print(f"  位置: {result['Pcmd_rad']:+.4f} rad ({result['Pcmd_rad']*180/3.14159:+.2f}°)")
    print(f"  速度: {result['Vcmd_rad_s']:+.3f} rad/s ({result['Vcmd_rad_s']*60/6.28318:+.1f} rpm)")
    print(f"  转矩: {result['Tcmd_Nm']:+.2f} N·m")

    print("\n=== 电流通道 ===")
    print(f"  Iq 指令: {result['iqref_A']:+.2f} A")
    print(f"  Iq 反馈: {result['iqfdb_A']:+.2f} A")
    print(f"  电流RMS: {result['Irms_A']:+.2f} A")
    print(f"  MIT t_ff: {result['MIT_T_Nm']:+.2f} N·m")

    print("\n=== 状态/故障 ===")
    print(f"  Err1: 0x{result['Err1']:04X}")
    print(f"  Err2: 0x{result['Err2']:04X}")
    print(f"  STA: 0x{result['STA']:02X}  [使能={result['sta_enabled']}, 故障={result['sta_fault']}, 警告={result['sta_warn']}, 到达={result['sta_arrived']}]")
    print(f"  warn: 0x{result['warn']:02X}  [MOS过温={result['warn_mos_overheat']}, 电机过温={result['warn_motor_overheat']}]")

    print("\n=== 辅助信号 ===")
    print(f"  母线电压: {result['Vdc_V']} V")
    print(f"  驱动板温度: {result['Temp_D_degC']:+.1f} °C")
    print(f"  电机温度: {result['Temp_M_degC']:+.1f} °C")

def main():
    parser = argparse.ArgumentParser(description='0x100+ID 状态帧解析工具')
    parser.add_argument('--hex', type=str, help='十六进制字符串 (空格分隔, 33字节)')
    parser.add_argument('--file', type=str, help='candump 日志文件')
    args = parser.parse_args()

    if args.hex:
        # 解析命令行十六进制字符串
        hex_str = args.hex.replace(' ', '').replace(',', '')
        if len(hex_str) % 2 != 0:
            print("Error: 十六进制字符串长度必须是偶数")
            sys.exit(1)
        data = bytes.fromhex(hex_str)
        result = parse_0x100(data)
        print_result(result)

    elif args.file:
        # 解析 candump 日志 (格式: "  (timestamp) vcan0  101   [48]  12 34 56 ...")
        with open(args.file, 'r') as f:
            for line in f:
                if '101' in line and '[48]' in line:  # 0x101 = 0x100 + ID=1
                    parts = line.split()
                    hex_data = ' '.join(parts[5:38])  # 取前 33 字节
                    try:
                        data = bytes.fromhex(hex_data)
                        result = parse_0x100(data)
                        print(f"\n[{parts[0]}] 帧时间: {parts[1]}")
                        print_result(result)
                    except Exception as e:
                        print(f"解析失败: {e}")
    else:
        print("请指定 --hex 或 --file 参数")
        sys.exit(1)

if __name__ == '__main__':
    main()
