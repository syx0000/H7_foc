"""Motor control panel for basic run/stop and mode selection.

Provides UI for sending Runcmd and enable commands.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QComboBox,
    QPushButton, QDoubleSpinBox, QGroupBox, QCheckBox
)
from PySide6.QtCore import Signal
from core.protocol import build_runcmd, build_enable


class MotorControlPanel(QWidget):
    """Motor control interface with mode selection and target input."""

    sig_command = Signal(str)  # Command string to send

    def __init__(self, parent=None):
        super().__init__(parent)

        # Two groups laid horizontally (Motor Control | Data Logging)
        # so the left tab page stays compact and the rest goes to waveform
        layout = QHBoxLayout(self)

        # Control group
        control_group = QGroupBox("Motor Control")
        control_layout = QVBoxLayout(control_group)

        # Mode selection
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItem("Position", 1)
        self._mode_combo.addItem("Velocity", 3)
        self._mode_combo.addItem("Torque", 4)
        self._mode_combo.setCurrentIndex(1)  # Default: Velocity
        mode_layout.addWidget(self._mode_combo)
        mode_layout.addStretch()
        control_layout.addLayout(mode_layout)

        # Target input + direction
        target_layout = QHBoxLayout()
        target_layout.addWidget(QLabel("Target:"))
        self._target_spin = QDoubleSpinBox()
        self._target_spin.setRange(0, 10000)
        self._target_spin.setDecimals(2)
        self._target_spin.setSuffix(" (°/rpm/Nm)")
        target_layout.addWidget(self._target_spin)

        self._reverse_cb = QCheckBox("Reverse")
        target_layout.addWidget(self._reverse_cb)

        target_layout.addStretch()
        control_layout.addLayout(target_layout)

        # Run/Stop buttons
        button_layout = QHBoxLayout()
        self._run_btn = QPushButton("Run")
        self._run_btn.clicked.connect(self._on_run_clicked)
        button_layout.addWidget(self._run_btn)

        self._enable_btn = QPushButton("Enable PWM")
        self._enable_btn.setCheckable(True)
        self._enable_btn.toggled.connect(self._on_enable_toggled)
        button_layout.addWidget(self._enable_btn)

        button_layout.addStretch()
        control_layout.addLayout(button_layout)
        control_layout.addStretch()

        layout.addWidget(control_group)

        # Log selection group
        log_group = QGroupBox("Data Logging")
        log_layout = QVBoxLayout(log_group)

        logid_layout = QHBoxLayout()
        logid_layout.addWidget(QLabel("Log ID:"))
        self._logid_combo = QComboBox()
        self._logid_combo.addItem("10 - Angle/Position", 10)
        self._logid_combo.addItem("30 - Voltage (Vq/Vd)", 30)
        self._logid_combo.addItem("40 - Current PI", 40)
        self._logid_combo.addItem("50 - Speed", 50)
        self._logid_combo.addItem("60 - CCR", 60)
        self._logid_combo.addItem("70 - Phase Current", 70)
        self._logid_combo.addItem("90 - Raw ADC", 90)
        self._logid_combo.addItem("100 - Position", 100)
        self._logid_combo.addItem("110 - ISR Timing", 110)
        self._logid_combo.setCurrentIndex(3)  # Default to logid 50 (speed)
        self._logid_combo.currentIndexChanged.connect(self._on_logid_changed)
        logid_layout.addWidget(self._logid_combo)

        # Enable/disable logging toggle
        self._log_enable_cb = QCheckBox("Enable")
        self._log_enable_cb.setChecked(False)
        self._log_enable_cb.toggled.connect(self._on_log_enable_toggled)
        logid_layout.addWidget(self._log_enable_cb)

        logfreq_layout = QHBoxLayout()
        logfreq_layout.addWidget(QLabel("Period (ms):"))
        self._logfreq_spin = QDoubleSpinBox()
        self._logfreq_spin.setRange(1, 1000)
        self._logfreq_spin.setValue(10)
        self._logfreq_spin.setDecimals(0)
        self._logfreq_spin.valueChanged.connect(self._on_logfreq_changed)
        logfreq_layout.addWidget(self._logfreq_spin)
        logfreq_layout.addStretch()

        log_layout.addLayout(logid_layout)
        log_layout.addLayout(logfreq_layout)
        log_layout.addStretch()

        layout.addWidget(log_group)
        # Both groups share width equally inside the horizontal layout

    def _on_run_clicked(self):
        """Send run command. Firmware auto-enables PWM on Run, so reflect that in UI."""
        mode = self._mode_combo.currentData()
        target = self._target_spin.value()
        if self._reverse_cb.isChecked():
            target = -target
        cmd = build_runcmd(cmd=2, mode=mode, target=target)
        self.sig_command.emit(cmd)
        # Sync UI: Run auto-enables PWM (TIM1->BDTR MOE + CCER), reflect without re-sending
        if not self._enable_btn.isChecked():
            self._enable_btn.blockSignals(True)
            self._enable_btn.setChecked(True)
            self._enable_btn.blockSignals(False)

    def _on_enable_toggled(self, checked: bool):
        """Send enable/disable command."""
        cmd = build_enable(checked)
        self.sig_command.emit(cmd)

    def _on_logid_changed(self):
        """Send logid command when selection changes (only if enabled)."""
        if not self._log_enable_cb.isChecked():
            return
        from core.protocol import build_logid
        logid = self._logid_combo.currentData()
        cmd = build_logid(logid)
        self.sig_command.emit(cmd)

    def _on_log_enable_toggled(self, checked: bool):
        """Enable/disable periodic logging."""
        from core.protocol import build_logid
        if checked:
            # Resume with selected logid
            logid = self._logid_combo.currentData()
            self.sig_command.emit(build_logid(logid))
        else:
            # Stop logging by sending logid0
            self.sig_command.emit(build_logid(0))

    def _on_logfreq_changed(self):
        """Send logfreq command when value changes."""
        from core.protocol import build_logfreq
        period_ms = int(self._logfreq_spin.value())
        cmd = build_logfreq(period_ms)
        self.sig_command.emit(cmd)

    def get_current_logid(self) -> int:
        """Get currently selected logid.

        Returns:
            Current logid value
        """
        return self._logid_combo.currentData()
