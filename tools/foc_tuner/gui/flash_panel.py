"""Flash parameter management panel.

Provides UI for reading, writing, erasing, and comparing Flash parameters.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QGroupBox, QTextEdit
)
from PySide6.QtCore import Signal


class FlashPanel(QWidget):
    """Flash parameter management interface."""

    sig_command = Signal(str)  # Command to send

    def __init__(self, parent=None):
        super().__init__(parent)
        self._in_dump_block = False  # Capturing FlashData Dump output

        layout = QVBoxLayout(self)

        # Control buttons
        button_group = QGroupBox("Flash Operations")
        button_layout = QHBoxLayout(button_group)

        self._read_btn = QPushButton("Read Flash (logid162)")
        self._read_btn.clicked.connect(self._on_read_clicked)
        button_layout.addWidget(self._read_btn)

        self._write_btn = QPushButton("Write to Flash (logid160)")
        self._write_btn.clicked.connect(self._on_write_clicked)
        button_layout.addWidget(self._write_btn)

        self._erase_btn = QPushButton("Erase Flash (logid161)")
        self._erase_btn.clicked.connect(self._on_erase_clicked)
        self._erase_btn.setStyleSheet("QPushButton { background-color: #ffcccc; }")
        button_layout.addWidget(self._erase_btn)

        button_layout.addStretch()
        layout.addWidget(button_group)

        # Info label
        info_label = QLabel(
            "Flash stores motor parameters (PID, offsets, limits, etc.).\n"
            "• Read: Compare RAM vs Flash (logid162)\n"
            "• Write: Save current RAM parameters to Flash (logid160)\n"
            "• Erase: Clear Flash sector - next boot will reinitialize (logid161)"
        )
        info_label.setWordWrap(True)
        layout.addWidget(info_label)

        # Result display
        result_group = QGroupBox("Flash Data")
        result_layout = QVBoxLayout(result_group)

        self._result_text = QTextEdit()
        self._result_text.setReadOnly(True)
        self._result_text.setStyleSheet("QTextEdit { font-family: Consolas, 'Courier New', monospace; }")
        result_layout.addWidget(self._result_text)

        layout.addWidget(result_group)

    def _on_read_clicked(self):
        """Send logid162 to read and compare Flash data."""
        self.sig_command.emit("logid162")
        self._result_text.append("\n>>> Reading Flash data...")

    def _on_write_clicked(self):
        """Send logid160 to write current parameters to Flash."""
        self.sig_command.emit("logid160")
        self._result_text.append("\n>>> Writing to Flash...")

    def _on_erase_clicked(self):
        """Send logid161 to erase Flash sector."""
        self.sig_command.emit("logid161")
        self._result_text.append("\n>>> Erasing Flash sector...")

    def process_line(self, line: str):
        """Process serial line for Flash-related output.

        Captures every line between "===== FlashData Dump" and "===== End ====="
        so all field rows (with leading spaces and numeric columns) are shown.

        Args:
            line: Text line from serial port
        """
        # Detect block start
        if "FlashData Dump" in line:
            self._in_dump_block = True
            self._result_text.append(line)
            return

        # Inside dump block: capture everything verbatim
        if self._in_dump_block:
            self._result_text.append(line)
            if "===== End =====" in line:
                self._in_dump_block = False
            return

        # Outside dump block: show standalone confirmations only
        if any(kw in line for kw in (
            "Flash erase OK", "Flash erase FAIL",
            "WriteDataToFlash", "faults cleared",
        )):
            self._result_text.append(line)
