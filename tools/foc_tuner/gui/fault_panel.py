"""Fault diagnosis panel for displaying and clearing fault flags.

Decodes ServoErrFlag bits and provides fault clearing functionality.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QTableWidget, QTableWidgetItem, QHeaderView
)
from PySide6.QtCore import Signal, Qt
from PySide6.QtGui import QColor


class FaultPanel(QWidget):
    """Fault diagnosis and clearing interface."""

    sig_command = Signal(str)  # Command to send

    # Fault bit definitions (from ifly_fault.c)
    FAULT_BITS = {
        0: ("OverBusVolErr", "母线过压"),
        1: ("LowBusVolErr", "母线欠压"),
        2: ("HighBoardTempErr", "板温过高"),
        3: ("OverBusCurrentErr", "母线过流"),
        4: ("HighMotorTempErr", "电机温度过高"),
        5: ("LockedRotorErr", "堵转"),
        6: ("PhaseUVolErr", "U相缺相"),
        7: ("PhaseVVolErr", "V相缺相"),
        8: ("PhaseWVolErr", "W相缺相"),
        9: ("DriverChipNfault", "驱动芯片nFAULT"),
    }

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QVBoxLayout(self)

        # Control buttons
        button_group = QGroupBox("Fault Control")
        button_layout = QHBoxLayout(button_group)

        self._clear_btn = QPushButton("Clear All Faults (logid163)")
        self._clear_btn.clicked.connect(self._on_clear_clicked)
        button_layout.addWidget(self._clear_btn)

        self._refresh_btn = QPushButton("Refresh Status")
        self._refresh_btn.clicked.connect(self._on_refresh_clicked)
        button_layout.addWidget(self._refresh_btn)

        button_layout.addStretch()
        layout.addWidget(button_group)

        # Fault status display
        status_group = QGroupBox("Fault Status")
        status_layout = QVBoxLayout(status_group)

        self._status_label = QLabel("No fault data received yet")
        self._status_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self._status_label)

        # Fault table
        self._fault_table = QTableWidget()
        self._fault_table.setColumnCount(3)
        self._fault_table.setHorizontalHeaderLabels(["Bit", "Fault Name", "Description"])
        self._fault_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self._fault_table.setRowCount(len(self.FAULT_BITS))

        for bit, (name, desc) in self.FAULT_BITS.items():
            self._fault_table.setItem(bit, 0, QTableWidgetItem(str(bit)))
            self._fault_table.setItem(bit, 1, QTableWidgetItem(name))
            self._fault_table.setItem(bit, 2, QTableWidgetItem(desc))

        status_layout.addWidget(self._fault_table)
        layout.addWidget(status_group)

        # Info label
        info_label = QLabel(
            "Fault flags are set when protection conditions are detected.\n"
            "Clear faults after resolving the root cause to re-enable motor operation."
        )
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

    def _on_clear_clicked(self):
        """Send logid163 to clear all fault flags."""
        self.sig_command.emit("logid163")

    def _on_refresh_clicked(self):
        """Request fault status (via logid165 if available, or just observe console)."""
        self.sig_command.emit("logid165")

    def process_line(self, line: str):
        """Process serial line for fault-related output.

        Args:
            line: Text line from serial port
        """
        # Parse "FAULT! ServoErrFlag=0x<hex>, PWM disabled"
        if "FAULT!" in line and "ServoErrFlag" in line:
            try:
                # Extract hex value
                parts = line.split("ServoErrFlag=")
                if len(parts) > 1:
                    hex_str = parts[1].split(",")[0].strip()
                    fault_code = int(hex_str, 16)
                    self._update_fault_display(fault_code)
            except Exception:
                pass

        # Parse fault clearing confirmation
        if "faults cleared" in line.lower():
            self._status_label.setText("✓ All faults cleared")
            self._status_label.setStyleSheet("QLabel { color: green; font-weight: bold; }")
            self._clear_fault_display()

    def _update_fault_display(self, fault_code: int):
        """Update fault table with active faults.

        Args:
            fault_code: ServoErrFlag value
        """
        self._status_label.setText(f"⚠ FAULT ACTIVE: 0x{fault_code:04X}")
        self._status_label.setStyleSheet("QLabel { color: red; font-weight: bold; }")

        # Highlight active faults
        for bit in range(16):
            if bit < len(self.FAULT_BITS):
                is_active = (fault_code >> bit) & 1
                color = QColor(255, 200, 200) if is_active else QColor(255, 255, 255)
                for col in range(3):
                    item = self._fault_table.item(bit, col)
                    if item:
                        item.setBackground(color)

    def _clear_fault_display(self):
        """Clear fault highlighting."""
        for bit in range(len(self.FAULT_BITS)):
            for col in range(3):
                item = self._fault_table.item(bit, col)
                if item:
                    item.setBackground(QColor(255, 255, 255))
