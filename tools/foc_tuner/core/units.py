"""Unit conversion utilities for FOC motor controller.

Units used in firmware:
- Position: 1°/1024 LSB (output end)
- Velocity: rpm × 1024 (motor end), output = motor/25
- Current: Q10 A (1024 = 1A)
- Voltage: Q10 V (1024 = 1V)
"""


def q10_to_float(value: int) -> float:
    """Convert Q10 fixed-point to float.

    Args:
        value: Q10 integer (1024 = 1.0)

    Returns:
        Float value in SI units
    """
    return value / 1024.0


def float_to_q10(value: float) -> int:
    """Convert float to Q10 fixed-point.

    Args:
        value: Float value in SI units

    Returns:
        Q10 integer (1024 = 1.0)
    """
    return int(value * 1024)


def deg_lsb_to_deg(value: int) -> float:
    """Convert position LSB to degrees.

    Args:
        value: Position in 1°/1024 LSB

    Returns:
        Position in degrees
    """
    return value / 1024.0


def deg_to_deg_lsb(value: float) -> int:
    """Convert degrees to position LSB.

    Args:
        value: Position in degrees

    Returns:
        Position in 1°/1024 LSB
    """
    return int(value * 1024)


def rpm_q10_to_rpm(value: int) -> float:
    """Convert velocity Q10 to rpm.

    Args:
        value: Velocity in rpm × 1024

    Returns:
        Velocity in rpm
    """
    return value / 1024.0


def rpm_to_rpm_q10(value: float) -> int:
    """Convert rpm to velocity Q10.

    Args:
        value: Velocity in rpm

    Returns:
        Velocity in rpm × 1024
    """
    return int(value * 1024)
