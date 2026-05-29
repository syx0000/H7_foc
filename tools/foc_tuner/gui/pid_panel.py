"""PID tuning panel for online parameter adjustment.

Provides sliders and spinboxes for tuning current/speed/position loop PIDs.
Supports saving/loading parameters to/from local JSON file.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox,
    QSlider, QPushButton, QGroupBox, QTabWidget, QFileDialog, QSpinBox
)
from PySide6.QtCore import Signal, Qt
import json
import os
import re


class PhaseCompTuner(QWidget):
    """Phase compensation tuner with 4 parameters."""

    sig_apply = Signal(int, int, int, int)  # offset_pos, offset_neg, comp_pos, comp_neg
    sig_save_flash = Signal()  # Save to Flash request

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Info label
        info = QLabel(
            "方向相关相位补偿:\n"
            "  theta_comp = offset × 18.2 counts + dtheta × comp / 10\n"
            "  offset: 固定偏置 (×0.1°)\n"
            "  comp: 速度相关 (×0.1 倍 dtheta)"
        )
        info.setStyleSheet("QLabel { color: gray; font-size: 10pt; }")
        layout.addWidget(info)

        # Offset positive
        offset_pos_layout = QHBoxLayout()
        offset_pos_layout.addWidget(QLabel("Offset Pos (×0.1°):"))
        self._offset_pos_spin = QSpinBox()
        self._offset_pos_spin.setRange(-3600, 3600)
        self._offset_pos_spin.setValue(0)
        offset_pos_layout.addWidget(self._offset_pos_spin)
        offset_pos_layout.addStretch()
        layout.addLayout(offset_pos_layout)

        # Offset negative
        offset_neg_layout = QHBoxLayout()
        offset_neg_layout.addWidget(QLabel("Offset Neg (×0.1°):"))
        self._offset_neg_spin = QSpinBox()
        self._offset_neg_spin.setRange(-3600, 3600)
        self._offset_neg_spin.setValue(0)
        offset_neg_layout.addWidget(self._offset_neg_spin)
        offset_neg_layout.addStretch()
        layout.addLayout(offset_neg_layout)

        # Comp positive
        comp_pos_layout = QHBoxLayout()
        comp_pos_layout.addWidget(QLabel("Comp Pos (×0.1):"))
        self._comp_pos_spin = QSpinBox()
        self._comp_pos_spin.setRange(-200, 200)
        self._comp_pos_spin.setValue(20)
        comp_pos_layout.addWidget(self._comp_pos_spin)
        comp_pos_layout.addStretch()
        layout.addLayout(comp_pos_layout)

        # Comp negative
        comp_neg_layout = QHBoxLayout()
        comp_neg_layout.addWidget(QLabel("Comp Neg (×0.1):"))
        self._comp_neg_spin = QSpinBox()
        self._comp_neg_spin.setRange(-200, 200)
        self._comp_neg_spin.setValue(26)
        comp_neg_layout.addWidget(self._comp_neg_spin)
        comp_neg_layout.addStretch()
        layout.addLayout(comp_neg_layout)

        # Buttons
        button_layout = QHBoxLayout()
        self._apply_btn = QPushButton("Apply to Motor")
        self._apply_btn.clicked.connect(self._on_apply_clicked)
        button_layout.addWidget(self._apply_btn)

        self._save_btn = QPushButton("Save to Flash")
        self._save_btn.clicked.connect(self._on_save_clicked)
        self._save_btn.setStyleSheet("QPushButton { background-color: #ccffcc; }")
        button_layout.addWidget(self._save_btn)

        button_layout.addStretch()
        layout.addLayout(button_layout)

        layout.addStretch()

    def _on_apply_clicked(self):
        """Emit signal when Apply button clicked."""
        self.sig_apply.emit(
            self._offset_pos_spin.value(),
            self._offset_neg_spin.value(),
            self._comp_pos_spin.value(),
            self._comp_neg_spin.value()
        )

    def _on_save_clicked(self):
        """Emit signal when Save to Flash button clicked."""
        self.sig_save_flash.emit()

    def get_values(self) -> tuple[int, int, int, int]:
        """Get current phase comp values."""
        return (
            self._offset_pos_spin.value(),
            self._offset_neg_spin.value(),
            self._comp_pos_spin.value(),
            self._comp_neg_spin.value()
        )

    def set_values(self, offset_pos: int, offset_neg: int, comp_pos: int, comp_neg: int):
        """Set phase comp values programmatically."""
        self._offset_pos_spin.setValue(offset_pos)
        self._offset_neg_spin.setValue(offset_neg)
        self._comp_pos_spin.setValue(comp_pos)
        self._comp_neg_spin.setValue(comp_neg)


class PIDTuner(QWidget):
    """Single PID loop tuner with Kp/Ki/Kd sliders."""

    sig_pid_changed = Signal(float, float, float)  # kp, ki, kd

    def __init__(self, title: str, kp_range: tuple, ki_range: tuple, kd_range: tuple,
                 kp_default: float = 0, ki_default: float = 0, kd_default: float = 0,
                 parent=None):
        super().__init__(parent)
        self._updating = False  # Prevent feedback loop

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)

        # Kp
        kp_layout = QHBoxLayout()
        kp_layout.addWidget(QLabel("Kp:"))
        self._kp_spin = QDoubleSpinBox()
        self._kp_spin.setRange(kp_range[0], kp_range[1])
        self._kp_spin.setValue(kp_default)
        self._kp_spin.setDecimals(1)
        self._kp_spin.valueChanged.connect(self._on_kp_changed)
        kp_layout.addWidget(self._kp_spin)

        self._kp_slider = QSlider(Qt.Horizontal)
        self._kp_slider.setRange(int(kp_range[0] * 10), int(kp_range[1] * 10))
        self._kp_slider.setValue(int(kp_default * 10))
        self._kp_slider.valueChanged.connect(self._on_kp_slider_changed)
        kp_layout.addWidget(self._kp_slider, 1)
        layout.addLayout(kp_layout)

        # Ki
        ki_layout = QHBoxLayout()
        ki_layout.addWidget(QLabel("Ki:"))
        self._ki_spin = QDoubleSpinBox()
        self._ki_spin.setRange(ki_range[0], ki_range[1])
        self._ki_spin.setValue(ki_default)
        self._ki_spin.setDecimals(1)
        self._ki_spin.valueChanged.connect(self._on_ki_changed)
        ki_layout.addWidget(self._ki_spin)

        self._ki_slider = QSlider(Qt.Horizontal)
        self._ki_slider.setRange(int(ki_range[0] * 10), int(ki_range[1] * 10))
        self._ki_slider.setValue(int(ki_default * 10))
        self._ki_slider.valueChanged.connect(self._on_ki_slider_changed)
        ki_layout.addWidget(self._ki_slider, 1)
        layout.addLayout(ki_layout)

        # Kd
        kd_layout = QHBoxLayout()
        kd_layout.addWidget(QLabel("Kd:"))
        self._kd_spin = QDoubleSpinBox()
        self._kd_spin.setRange(kd_range[0], kd_range[1])
        self._kd_spin.setValue(kd_default)
        self._kd_spin.setDecimals(1)
        self._kd_spin.valueChanged.connect(self._on_kd_changed)
        kd_layout.addWidget(self._kd_spin)

        self._kd_slider = QSlider(Qt.Horizontal)
        self._kd_slider.setRange(int(kd_range[0] * 10), int(kd_range[1] * 10))
        self._kd_slider.setValue(int(kd_default * 10))
        self._kd_slider.valueChanged.connect(self._on_kd_slider_changed)
        kd_layout.addWidget(self._kd_slider, 1)
        layout.addLayout(kd_layout)

        # Apply button
        self._apply_btn = QPushButton("Apply to Motor")
        self._apply_btn.clicked.connect(self._on_apply_clicked)
        layout.addWidget(self._apply_btn)

    def _on_kp_changed(self, value: float):
        if not self._updating:
            self._updating = True
            self._kp_slider.setValue(int(value * 10))
            self._updating = False

    def _on_kp_slider_changed(self, value: int):
        if not self._updating:
            self._updating = True
            self._kp_spin.setValue(value / 10.0)
            self._updating = False

    def _on_ki_changed(self, value: float):
        if not self._updating:
            self._updating = True
            self._ki_slider.setValue(int(value * 10))
            self._updating = False

    def _on_ki_slider_changed(self, value: int):
        if not self._updating:
            self._updating = True
            self._ki_spin.setValue(value / 10.0)
            self._updating = False

    def _on_kd_changed(self, value: float):
        if not self._updating:
            self._updating = True
            self._kd_slider.setValue(int(value * 10))
            self._updating = False

    def _on_kd_slider_changed(self, value: int):
        if not self._updating:
            self._updating = True
            self._kd_spin.setValue(value / 10.0)
            self._updating = False

    def _on_apply_clicked(self):
        """Emit signal when Apply button clicked."""
        self.sig_pid_changed.emit(
            self._kp_spin.value(),
            self._ki_spin.value(),
            self._kd_spin.value()
        )

    def get_values(self) -> tuple[float, float, float]:
        """Get current PID values."""
        return (self._kp_spin.value(), self._ki_spin.value(), self._kd_spin.value())

    def set_values(self, kp: float, ki: float, kd: float):
        """Set PID values programmatically."""
        self._updating = True
        self._kp_spin.setValue(kp)
        self._kp_slider.setValue(int(kp * 10))
        self._ki_spin.setValue(ki)
        self._ki_slider.setValue(int(ki * 10))
        self._kd_spin.setValue(kd)
        self._kd_slider.setValue(int(kd * 10))
        self._updating = False


class PIDPanel(QWidget):
    """Complete PID tuning panel with all three loops and save/load."""

    sig_command = Signal(str)  # Command to send to motor

    def __init__(self, parent=None):
        super().__init__(parent)
        self._config_file = "foc_tuner_config.json"
        self._collecting_params = False  # Inside PARAMS_BEGIN/END block

        layout = QVBoxLayout(self)

        # Tab widget for three loops
        tabs = QTabWidget()

        # Current loop (实测最佳值: Kp=45, Ki=4, Kd=0)
        self._current_tuner = PIDTuner(
            "Current Loop",
            kp_range=(0, 200), ki_range=(0, 50), kd_range=(0, 10),
            kp_default=45, ki_default=4, kd_default=0
        )
        self._current_tuner.sig_pid_changed.connect(self._on_current_pid_changed)
        tabs.addTab(self._current_tuner, "Current Loop")

        # Speed loop (实测最佳值: Kp=1500, Ki=10, Kd=0)
        self._speed_tuner = PIDTuner(
            "Speed Loop",
            kp_range=(0, 5000), ki_range=(0, 100), kd_range=(0, 50),
            kp_default=1500, ki_default=10, kd_default=0
        )
        self._speed_tuner.sig_pid_changed.connect(self._on_speed_pid_changed)
        tabs.addTab(self._speed_tuner, "Speed Loop")

        # Position loop (实测最佳值: Kp=3016, Ki=9, Kd=0)
        self._position_tuner = PIDTuner(
            "Position Loop",
            kp_range=(0, 10000), ki_range=(0, 100), kd_range=(0, 50),
            kp_default=3016, ki_default=9, kd_default=0
        )
        self._position_tuner.sig_pid_changed.connect(self._on_position_pid_changed)
        tabs.addTab(self._position_tuner, "Position Loop")

        # Phase compensation (实测默认值: offset_pos=0, offset_neg=0, comp_pos=20, comp_neg=26)
        self._phase_comp_tuner = PhaseCompTuner()
        self._phase_comp_tuner.sig_apply.connect(self._on_phase_comp_apply)
        self._phase_comp_tuner.sig_save_flash.connect(self._on_phase_comp_save_flash)
        tabs.addTab(self._phase_comp_tuner, "Phase Comp")

        layout.addWidget(tabs)

        # Save/Load buttons
        button_layout = QHBoxLayout()

        read_btn = QPushButton("Read from MCU")
        read_btn.clicked.connect(self.query_params)
        read_btn.setStyleSheet("QPushButton { background-color: #cceeff; }")
        button_layout.addWidget(read_btn)

        save_btn = QPushButton("Save Config")
        save_btn.clicked.connect(self._on_save_clicked)
        button_layout.addWidget(save_btn)

        load_btn = QPushButton("Load Config")
        load_btn.clicked.connect(self._on_load_clicked)
        button_layout.addWidget(load_btn)

        self._status_label = QLabel("")
        self._status_label.setStyleSheet("QLabel { color: gray; font-size: 9pt; }")
        button_layout.addWidget(self._status_label)

        button_layout.addStretch()
        layout.addLayout(button_layout)

        # Auto-load config on startup
        self._load_config()

    def _on_current_pid_changed(self, kp: float, ki: float, kd: float):
        """Send CurrentPID command."""
        from core.protocol import build_current_pid
        cmd = build_current_pid(kp, ki, kd)
        self.sig_command.emit(cmd)

    def _on_speed_pid_changed(self, kp: float, ki: float, kd: float):
        """Send SpeedPID command."""
        from core.protocol import build_speed_pid
        cmd = build_speed_pid(kp, ki, kd)
        self.sig_command.emit(cmd)

    def _on_position_pid_changed(self, kp: float, ki: float, kd: float):
        """Send PositionPID command."""
        from core.protocol import build_position_pid
        cmd = build_position_pid(kp, ki, kd)
        self.sig_command.emit(cmd)

    def _on_phase_comp_apply(self, offset_pos: int, offset_neg: int, comp_pos: int, comp_neg: int):
        """Send phase compensation commands."""
        from core.protocol import build_phase_comp
        cmd = build_phase_comp(offset_pos, offset_neg, comp_pos, comp_neg)
        self.sig_command.emit(cmd)

    def _on_phase_comp_save_flash(self):
        """Send savephasecomp command to persist to Flash."""
        from core.protocol import build_save_phase_comp
        cmd = build_save_phase_comp()
        self.sig_command.emit(cmd)

    def query_params(self):
        """Send getparams command to MCU. Response is parsed in process_line()."""
        from core.protocol import build_query_params
        self._collecting_params = False  # Will be set True on PARAMS_BEGIN
        self._status_label.setText("Reading from MCU...")
        self._status_label.setStyleSheet("QLabel { color: orange; font-size: 9pt; }")
        self.sig_command.emit(build_query_params())

    def process_line(self, line: str):
        """Process serial line for parameter query response.

        Expected format:
            PARAMS_BEGIN
            CurKp=45 CurKi=4 CurKd=0
            SpdKp=1500 SpdKi=10 SpdKd=0
            PosKp=3016 PosKi=9 PosKd=0
            OffPos=0 OffNeg=0 CompPos=20 CompNeg=26
            PARAMS_END
        """
        if "PARAMS_BEGIN" in line:
            self._collecting_params = True
            return

        if not self._collecting_params:
            return

        if "PARAMS_END" in line:
            self._collecting_params = False
            self._status_label.setText("✓ Loaded from MCU")
            self._status_label.setStyleSheet("QLabel { color: green; font-size: 9pt; }")
            return

        # Parse key=value pairs (signed integers, including negatives)
        pairs = dict(re.findall(r"(\w+)=(-?\d+)", line))
        if not pairs:
            return

        try:
            if "CurKp" in pairs:
                self._current_tuner.set_values(
                    float(pairs["CurKp"]), float(pairs["CurKi"]), float(pairs["CurKd"])
                )
            elif "SpdKp" in pairs:
                self._speed_tuner.set_values(
                    float(pairs["SpdKp"]), float(pairs["SpdKi"]), float(pairs["SpdKd"])
                )
            elif "PosKp" in pairs:
                self._position_tuner.set_values(
                    float(pairs["PosKp"]), float(pairs["PosKi"]), float(pairs["PosKd"])
                )
            elif "OffPos" in pairs:
                self._phase_comp_tuner.set_values(
                    int(pairs["OffPos"]), int(pairs["OffNeg"]),
                    int(pairs["CompPos"]), int(pairs["CompNeg"])
                )
        except (KeyError, ValueError):
            pass

    def _on_save_clicked(self):
        """Save current PID values to JSON file."""
        config = {
            "current_loop": {
                "kp": self._current_tuner.get_values()[0],
                "ki": self._current_tuner.get_values()[1],
                "kd": self._current_tuner.get_values()[2],
            },
            "speed_loop": {
                "kp": self._speed_tuner.get_values()[0],
                "ki": self._speed_tuner.get_values()[1],
                "kd": self._speed_tuner.get_values()[2],
            },
            "position_loop": {
                "kp": self._position_tuner.get_values()[0],
                "ki": self._position_tuner.get_values()[1],
                "kd": self._position_tuner.get_values()[2],
            },
            "phase_comp": {
                "offset_pos": self._phase_comp_tuner.get_values()[0],
                "offset_neg": self._phase_comp_tuner.get_values()[1],
                "comp_pos": self._phase_comp_tuner.get_values()[2],
                "comp_neg": self._phase_comp_tuner.get_values()[3],
            },
        }

        try:
            with open(self._config_file, 'w') as f:
                json.dump(config, f, indent=2)
            print(f"Config saved to {self._config_file}")
        except Exception as e:
            print(f"Failed to save config: {e}")

    def _on_load_clicked(self):
        """Load PID values from JSON file."""
        self._load_config()

    def _load_config(self):
        """Load config from file if it exists."""
        if not os.path.exists(self._config_file):
            return

        try:
            with open(self._config_file, 'r') as f:
                config = json.load(f)

            if "current_loop" in config:
                c = config["current_loop"]
                self._current_tuner.set_values(c["kp"], c["ki"], c["kd"])

            if "speed_loop" in config:
                s = config["speed_loop"]
                self._speed_tuner.set_values(s["kp"], s["ki"], s["kd"])

            if "position_loop" in config:
                p = config["position_loop"]
                self._position_tuner.set_values(p["kp"], p["ki"], p["kd"])

            if "phase_comp" in config:
                pc = config["phase_comp"]
                self._phase_comp_tuner.set_values(
                    pc["offset_pos"], pc["offset_neg"], pc["comp_pos"], pc["comp_neg"]
                )

            print(f"Config loaded from {self._config_file}")
        except Exception as e:
            print(f"Failed to load config: {e}")
