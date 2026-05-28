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
        target: Target value (position in °, velocity in rpm, torque in A)

    Returns:
        Command string (e.g., "Runcmd2M3tar20")
    """
    return f"Runcmd{cmd}M{mode}tar{int(target)}"


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
