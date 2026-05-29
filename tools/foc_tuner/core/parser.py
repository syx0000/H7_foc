"""Response line parser for FOC motor controller serial protocol.

Parses text lines from firmware printf output into structured data frames.
Uses a registry pattern for extensibility - each logid format is a separate function.
"""

import re
import time
from dataclasses import dataclass
from typing import Callable


@dataclass
class ParsedFrame:
    """Structured data frame from a parsed log line."""
    timestamp: float         # time.perf_counter() on receive
    logid: int               # which log type
    fields: dict[str, float] # named values in SI units (A, V, °, rpm)


# Registry of line parsers: prefix -> parser function
_PARSERS: dict[str, Callable[[str], ParsedFrame | None]] = {}


def register(prefix: str):
    """Decorator to register a line parser.

    Args:
        prefix: String prefix to match in the line (e.g., "current_pi:")

    Returns:
        Decorator function
    """
    def decorator(fn):
        _PARSERS[prefix] = fn
        return fn
    return decorator


@register("Angle_elec_360:")
def _parse_angle_elec(line: str) -> ParsedFrame | None:
    """Parse logid 10: Angle_elec_360: %d, %d, %d, %d, %d"""
    m = re.match(r'Angle_elec_360:\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)', line)
    if not m:
        return None
    vals = [int(x) for x in m.groups()]
    return ParsedFrame(
        timestamp=time.perf_counter(),
        logid=10,
        fields={
            'now_mechposition': vals[0] / 1024.0,  # degrees
            'theta_elec': vals[1],                  # 0-65536
            'real_position_out': vals[2] / 1024.0,  # degrees
            'real_position': vals[3] / 1024.0,      # degrees
            'dtheta_mech_rpm': vals[4],             # rpm (already /1024 in firmware)
        }
    )


@register("current_get:")
def _parse_voltage(line: str) -> ParsedFrame | None:
    """Parse logid 30: current_get: %d,%d (V_q, V_d)"""
    m = re.match(r'current_get:\s*(-?\d+),\s*(-?\d+)', line)
    if not m:
        return None
    vals = [int(x) for x in m.groups()]
    return ParsedFrame(
        timestamp=time.perf_counter(),
        logid=30,
        fields={
            'V_q': vals[0] / 1024.0,  # V
            'V_d': vals[1] / 1024.0,  # V
        }
    )


@register("current_pi:")
def _parse_current_pi(line: str) -> ParsedFrame | None:
    """Parse logid 40: current_pi: %d, %d, %d, %d, %d, %d, %d"""
    m = re.match(r'current_pi:\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)', line)
    if not m:
        return None
    vals = [int(x) for x in m.groups()]
    return ParsedFrame(
        timestamp=time.perf_counter(),
        logid=40,
        fields={
            'I_q': vals[0] / 1024.0,           # A
            'I_d': vals[1] / 1024.0,           # A
            'V_q': vals[2] / 1024.0,           # V
            'V_d': vals[3] / 1024.0,           # V
            'I_q_ref': vals[4] / 1024.0,       # A
            'I_d_ref': vals[5] / 1024.0,       # A
            'I_q_ref_filt': vals[6] / 1024.0,  # A
        }
    )


@register("speed:")
def _parse_speed(line: str) -> ParsedFrame | None:
    """Parse logid 50: speed: %d, %d, %d, %d, %d"""
    m = re.match(r'speed:\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)', line)
    if not m:
        return None
    vals = [int(x) for x in m.groups()]
    return ParsedFrame(
        timestamp=time.perf_counter(),
        logid=50,
        fields={
            'vel_ref': vals[0],           # rpm (already /1024 in firmware)
            'vel_ref_filt': vals[1],      # rpm
            'dtheta_mech': vals[2],       # rpm (motor end)
            'dtheta_mech_load_eq': vals[3],   # rpm (motor end /GR, 等效负载端)
            'vel_diff': vals[4],          # rpm (vel_ref - dtheta_mech)
        }
    )


@register("CCR")
def _parse_ccr_unused(line: str) -> ParsedFrame | None:
    """Placeholder - CCR logs have no prefix, dispatched by _parse_numeric_line."""
    return None


# Currently active logid - set by GUI when user changes logid selection.
# Used to disambiguate prefix-less numeric lines (logid 60/70/90 all use pure digits).
_active_logid: int = 0


def set_active_logid(logid: int):
    """Tell parser which logid is currently expected.

    Required for prefix-less numeric formats (60=CCR, 70=phase_current, 90=raw_adc)
    which all share the form '%d, %d, %d[, %d, %d, %d]' with no header.

    Args:
        logid: Currently active logid as set in firmware
    """
    global _active_logid
    _active_logid = logid


def _parse_numeric_line(line: str) -> ParsedFrame | None:
    """Parse prefix-less numeric lines based on active logid.

    Handles:
    - logid 60: 'CCR2, CCR3, CCR4'
    - logid 70: 'CCR2, CCR3, CCR4, I_a, I_b, I_c'
    - logid 90: 'Ia_raw, Ib_raw, Ic_raw'
    """
    if _active_logid == 70:
        m = re.match(r'^(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+),\s*(-?\d+)$', line)
        if not m:
            return None
        vals = [int(x) for x in m.groups()]
        return ParsedFrame(
            timestamp=time.perf_counter(),
            logid=70,
            fields={
                'CCR2': vals[0], 'CCR3': vals[1], 'CCR4': vals[2],
                'I_a': vals[3] / 1024.0,
                'I_b': vals[4] / 1024.0,
                'I_c': vals[5] / 1024.0,
            }
        )

    if _active_logid in (60, 90):
        m = re.match(r'^(-?\d+),\s*(-?\d+),\s*(-?\d+)$', line)
        if not m:
            return None
        vals = [int(x) for x in m.groups()]
        if _active_logid == 90:
            return ParsedFrame(
                timestamp=time.perf_counter(),
                logid=90,
                fields={'Ia_raw': vals[0], 'Ib_raw': vals[1], 'Ic_raw': vals[2]}
            )
        return ParsedFrame(
            timestamp=time.perf_counter(),
            logid=60,
            fields={'CCR2': vals[0], 'CCR3': vals[1], 'CCR4': vals[2]}
        )

    return None


@register("position:")
def _parse_position(line: str) -> ParsedFrame | None:
    """Parse logid 100: position: %f, %f, %f, %d"""
    m = re.match(r'position:\s*([-\d.]+),\s*([-\d.]+),\s*([-\d.]+),\s*(-?\d+)', line)
    if not m:
        return None
    return ParsedFrame(
        timestamp=time.perf_counter(),
        logid=100,
        fields={
            'pos_ref': float(m.group(1)),      # degrees
            'pos_out': float(m.group(2)),      # degrees
            'pos_error': float(m.group(3)),    # degrees
            'mech_offset': int(m.group(4)),    # LSB
        }
    )


@register("adc_isr_us")
def _parse_isr_timing(line: str) -> ParsedFrame | None:
    """Parse logid 110: adc_isr_us tot:%lu/%lu read:%lu/%lu enc:%lu/%lu pos:%lu/%lu vel:%lu/%lu cur:%lu/%lu"""
    m = re.match(
        r'adc_isr_us tot:(\d+)/(\d+) read:(\d+)/(\d+) enc:(\d+)/(\d+) '
        r'pos:(\d+)/(\d+) vel:(\d+)/(\d+) cur:(\d+)/(\d+)',
        line
    )
    if not m:
        return None
    vals = [int(x) for x in m.groups()]
    return ParsedFrame(
        timestamp=time.perf_counter(),
        logid=110,
        fields={
            'tot_us': vals[0],
            'tot_us_max': vals[1],
            'read_us': vals[2],
            'read_us_max': vals[3],
            'enc_us': vals[4],
            'enc_us_max': vals[5],
            'pos_us': vals[6],
            'pos_us_max': vals[7],
            'vel_us': vals[8],
            'vel_us_max': vals[9],
            'cur_us': vals[10],
            'cur_us_max': vals[11],
        }
    )


def parse_line(line: str) -> ParsedFrame | None:
    """Parse a line from the serial port.

    Args:
        line: Text line (stripped of \\r\\n)

    Returns:
        ParsedFrame if line matches a known format, None otherwise
    """
    for prefix, parser_fn in _PARSERS.items():
        if prefix in line:
            try:
                result = parser_fn(line)
                if result is not None:
                    return result
            except Exception:
                return None

    # Fallback: prefix-less numeric line (logid 60/70/90)
    return _parse_numeric_line(line)


def get_channel_names(logid: int) -> list[str]:
    """Get list of channel names for a given logid.

    Args:
        logid: Log ID

    Returns:
        List of channel names that will appear in ParsedFrame.fields
    """
    channel_map = {
        10: ['now_mechposition', 'theta_elec', 'real_position_out', 'real_position', 'dtheta_mech_rpm'],
        30: ['V_q', 'V_d'],
        40: ['I_q', 'I_d', 'V_q', 'V_d', 'I_q_ref', 'I_d_ref', 'I_q_ref_filt'],
        50: ['vel_ref', 'vel_ref_filt', 'dtheta_mech', 'dtheta_mech_load_eq', 'vel_diff'],
        60: ['CCR2', 'CCR3', 'CCR4'],
        70: ['CCR2', 'CCR3', 'CCR4', 'I_a', 'I_b', 'I_c'],
        90: ['Ia_raw', 'Ib_raw', 'Ic_raw'],
        100: ['pos_ref', 'pos_out', 'pos_error', 'mech_offset'],
        110: ['tot_us', 'tot_us_max', 'read_us', 'read_us_max', 'enc_us', 'enc_us_max',
              'pos_us', 'pos_us_max', 'vel_us', 'vel_us_max', 'cur_us', 'cur_us_max'],
    }
    return channel_map.get(logid, [])
