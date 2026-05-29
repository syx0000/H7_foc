"""Command protocol builders for FOC motor controller.

Generates command strings to send to the STM32 firmware via serial.
"""


def build_logid(logid: int) -> str:
    """Build logid command to select periodic log output.

    Args:
        logid: Log ID (10=angle, 30=voltage, 40=current_pi, 50=speed, etc.)

    Returns:
        Command string (e.g., "logid40")
    """
    return f"logid{logid}"


def build_logfreq(period_ms: int) -> str:
    """Build logfreq command to set log period.

    Args:
        period_ms: Log period in milliseconds

    Returns:
        Command string (e.g., "logfreq10")
    """
    return f"logfreq{period_ms}"


def build_current_pid(kp: float, ki: float, kd: float) -> str:
    """Build CurrentPID command.

    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain

    Returns:
        Command string (e.g., "CurrentPIDKp45Ki4Kd0")
    """
    return f"CurrentPIDKp{int(kp)}Ki{int(ki)}Kd{int(kd)}"


def build_speed_pid(kp: float, ki: float, kd: float) -> str:
    """Build SpeedPID command.

    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain

    Returns:
        Command string (e.g., "SpeedPIDKp1500Ki10Kd0")
    """
    return f"SpeedPIDKp{int(kp)}Ki{int(ki)}Kd{int(kd)}"


def build_position_pid(kp: float, ki: float, kd: float) -> str:
    """Build PositionPID command.

    Args:
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain

    Returns:
        Command string (e.g., "PositionPIDKp3000Ki9Kd0")
    """
    return f"PositionPIDKp{int(kp)}Ki{int(ki)}Kd{int(kd)}"


def build_runcmd(cmd: int, mode: int, target: float) -> str:
    """Build Runcmd command for motor control.

    Args:
        cmd: Command (0=stop, 2=run)
        mode: Control mode (1=position, 3=velocity, 4=torque, 8=CSP, 9=CSV, 10=CST)
        target: Target value (position in °, velocity in rpm, torque in Nm)

    Returns:
        Command string (e.g., "Runcmd2M3tar20.5")
    """
    return f"Runcmd{cmd}M{mode}tar{target}"


def build_enable(enable: bool) -> str:
    """Build enable command to control PWM output.

    Args:
        enable: True to enable PWM, False to disable

    Returns:
        Command string (e.g., "enable1" or "enable0")
    """
    return f"enable{1 if enable else 0}"


def build_cali() -> str:
    """Build Cali command for electrical angle calibration.

    Returns:
        Command string "Cali"
    """
    return "Cali"


def build_version() -> str:
    """Build version command to query firmware version.

    Returns:
        Command string "version"
    """
    return "version"


def build_bwtest(test_id: int) -> str:
    """Build bwtest command for bandwidth test or identification.

    Args:
        test_id: Test ID (1=current_bw, 2=speed_bw, 3=Rs/Ld/Lq, 4=flux, 5=inertia,
                         6=current_autotune, 7=speed_autotune, 8=position_autotune,
                         9=position_bw, 10=deadtime_cal)

    Returns:
        Command string (e.g., "bwtest1")
    """
    return f"bwtest{test_id}"


def build_flash_write() -> str:
    """Build command to write parameters to Flash.

    Returns:
        Command string "logid160"
    """
    return "logid160"


def build_flash_erase() -> str:
    """Build command to erase Flash sector.

    Returns:
        Command string "logid161"
    """
    return "logid161"


def build_flash_compare() -> str:
    """Build command to compare RAM vs Flash parameters.

    Returns:
        Command string "logid162"
    """
    return "logid162"


def build_clear_faults() -> str:
    """Build command to clear all fault flags.

    Returns:
        Command string "logid163"
    """
    return "logid163"


def build_reset() -> str:
    """Build command to trigger MCU system reset (NVIC_SystemReset).

    Returns:
        Command string "reset"
    """
    return "reset"


def build_phase_comp(offset_pos: int, offset_neg: int, comp_pos: int, comp_neg: int) -> str:
    """Build phase compensation commands (4 separate commands).

    Args:
        offset_pos: Forward rotation fixed angle offset (×0.1°)
        offset_neg: Reverse rotation fixed angle offset (×0.1°)
        comp_pos: Forward rotation speed-related compensation (×0.1)
        comp_neg: Reverse rotation speed-related compensation (×0.1)

    Returns:
        Four command strings joined by newlines
    """
    return (f"offsetpos{offset_pos}\r\n"
            f"offsetneg{offset_neg}\r\n"
            f"comppos{comp_pos}\r\n"
            f"compneg{comp_neg}")


def build_save_phase_comp() -> str:
    """Build command to save phase compensation parameters to Flash.

    Returns:
        Command string "savephasecomp"
    """
    return "savephasecomp"


def build_query_params() -> str:
    """Build command to query current PID + phase comp parameters.

    Firmware responds with PARAMS_BEGIN ... PARAMS_END block.

    Returns:
        Command string "getparams"
    """
    return "getparams"


# ============================================================================
# OTA firmware upgrade protocol
# ============================================================================
#
# Wire format:
#   Control frames (text, '\r\n' terminated):
#     otabegin SIZE=<n> CRC=0x<hex> VER=<v>     -- start, MCU erases App-B
#     OTA_READY chunk=256                        -- MCU ack, ready to receive
#     otaend                                    -- finalize, MCU verifies + writes header
#     OTA_DONE / OTA_FAIL <reason>              -- final result
#     otaabort                                  -- cancel + clear App-B header
#     OTA_ACK <seq> / OTA_NAK <seq> <reason>    -- per-chunk ack
#     OTA_ERR <reason>                          -- generic error
#
#   Data frames (binary, fixed header):
#     'OD' (2)  + seq:u16 LE + len:u16 LE + payload(len bytes) + crc16:u16 LE
#     CRC16 is MODBUS (poly=0xA001 reflected, init=0xFFFF, no xorOut),
#     covers header + payload (everything before the trailing CRC).
#
# Why 256B payload: USART1 RX DMA buffer on the MCU is 128B and is reused
# every IDLE event; in OTA mode the RX ISR streams bytes directly into a
# separate ring buffer, but keeping chunks small bounds re-transmit cost
# and matches an integer multiple of the H7 32B Flash write granularity.

_OTA_CHUNK_SIZE = 256


def build_ota_begin(size: int, crc32: int, version: str = "1.0.0") -> str:
    """Begin an OTA session.

    Args:
        size: Firmware byte count (must fit in App-B slot)
        crc32: IEEE 802.3 CRC32 of the entire firmware (matches Flash_Crc32)
        version: Free-form version string, no spaces

    Returns:
        Command string for the begin frame
    """
    return f"otabegin SIZE={size} CRC=0x{crc32:08X} VER={version}"


def build_ota_end() -> str:
    """Finalize OTA session — MCU verifies whole-image CRC and writes header."""
    return "otaend"


def build_ota_abort() -> str:
    """Abort OTA session — MCU clears App-B header to mark slot invalid."""
    return "otaabort"


def build_ota_swap() -> str:
    """Reboot into bootloader so it can swap to the newly written slot."""
    return "otaswap"


def crc16_modbus(data: bytes) -> int:
    """CRC16-MODBUS (poly=0xA001 reflected, init=0xFFFF, no xorOut).

    Verified against the standard test vector b'123456789' -> 0x4B37.
    """
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_ota_data_frame(seq: int, payload: bytes) -> bytes:
    """Build a binary OTA data frame.

    Frame layout (little-endian):
        offset  size  field
        0       2     'OD' magic
        2       2     seq
        4       2     len (== len(payload))
        6       len   payload
        6+len   2     crc16 over bytes [0 .. 6+len)
    """
    if len(payload) > _OTA_CHUNK_SIZE:
        raise ValueError(f"payload {len(payload)} > chunk size {_OTA_CHUNK_SIZE}")
    import struct
    header = b'OD' + struct.pack('<HH', seq & 0xFFFF, len(payload))
    body = header + payload
    crc = crc16_modbus(body)
    return body + struct.pack('<H', crc)


def get_ota_chunk_size() -> int:
    """Return the negotiated OTA payload size."""
    return _OTA_CHUNK_SIZE
