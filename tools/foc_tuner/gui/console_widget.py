"""Console widget for displaying raw serial output.

Append-only text viewer with line limit, clear, save to file, and a
custom command input row. SerialWorker.send() auto-appends \\r\\n.
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QPushButton,
    QFileDialog, QLineEdit, QLabel
)
from PySide6.QtCore import Signal
from PySide6.QtGui import QFont, QKeyEvent
from datetime import datetime


class _HistoryLineEdit(QLineEdit):
    """QLineEdit with Up/Down arrow command history navigation."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._history: list[str] = []
        self._history_idx = 0  # points to one past the end (= "current draft")
        self._draft = ""

    def push_history(self, cmd: str):
        if cmd and (not self._history or self._history[-1] != cmd):
            self._history.append(cmd)
            if len(self._history) > 100:
                self._history.pop(0)
        self._history_idx = len(self._history)
        self._draft = ""

    def keyPressEvent(self, event: QKeyEvent):
        from PySide6.QtCore import Qt
        if event.key() == Qt.Key_Up and self._history:
            if self._history_idx == len(self._history):
                self._draft = self.text()
            self._history_idx = max(0, self._history_idx - 1)
            self.setText(self._history[self._history_idx])
            return
        if event.key() == Qt.Key_Down and self._history:
            if self._history_idx >= len(self._history):
                return
            self._history_idx += 1
            if self._history_idx == len(self._history):
                self.setText(self._draft)
            else:
                self.setText(self._history[self._history_idx])
            return
        super().keyPressEvent(event)


class ConsoleWidget(QWidget):
    """Raw text log viewer with auto-scroll and line limit."""

    sig_send_command = Signal(str)  # User-typed custom command (no \r\n)

    def __init__(self, max_lines: int = 5000, parent=None):
        super().__init__(parent)
        self._max_lines = max_lines

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Toolbar
        toolbar = QHBoxLayout()

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self.clear)
        self._clear_btn.setMaximumWidth(80)
        toolbar.addWidget(self._clear_btn)

        self._save_btn = QPushButton("Save Log...")
        self._save_btn.clicked.connect(self._on_save_clicked)
        self._save_btn.setMaximumWidth(100)
        toolbar.addWidget(self._save_btn)

        toolbar.addStretch()
        layout.addLayout(toolbar)

        self._text_edit = QPlainTextEdit()
        self._text_edit.setReadOnly(True)
        self._text_edit.setMaximumBlockCount(max_lines)
        self._text_edit.setFont(QFont("Consolas", 9))
        layout.addWidget(self._text_edit)

        # Custom command input row
        cmd_row = QHBoxLayout()
        cmd_row.addWidget(QLabel("Cmd:"))

        self._cmd_input = _HistoryLineEdit()
        self._cmd_input.setFont(QFont("Consolas", 9))
        self._cmd_input.setPlaceholderText("Type command (auto \\r\\n on send)  -  Up/Down for history")
        self._cmd_input.returnPressed.connect(self._on_send_clicked)
        cmd_row.addWidget(self._cmd_input)

        self._send_btn = QPushButton("Send")
        self._send_btn.setMaximumWidth(80)
        self._send_btn.clicked.connect(self._on_send_clicked)
        cmd_row.addWidget(self._send_btn)

        layout.addLayout(cmd_row)

        # Disabled until serial connects
        self.set_send_enabled(False)

    def append_line(self, line: str):
        """Append a line to the console.

        Args:
            line: Text line (without \\r\\n)
        """
        self._text_edit.appendPlainText(line)

    def clear(self):
        """Clear all text."""
        self._text_edit.clear()

    def _on_save_clicked(self):
        """Save console log to file."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        default_name = f"foc_log_{timestamp}.txt"

        file_path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Console Log",
            default_name,
            "Text Files (*.txt);;Log Files (*.log);;All Files (*.*)"
        )

        if not file_path:
            return

        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(self._text_edit.toPlainText())
            # Optionally show success feedback in console
            self.append_line(f">>> Log saved to: {file_path}")
        except Exception as e:
            self.append_line(f">>> ERROR: Failed to save log: {e}")

    def set_send_enabled(self, enabled: bool):
        """Enable/disable the custom command input row."""
        self._cmd_input.setEnabled(enabled)
        self._send_btn.setEnabled(enabled)

    def _on_send_clicked(self):
        """Send the typed command (SerialWorker auto-appends \\r\\n)."""
        cmd = self._cmd_input.text().strip()
        if not cmd:
            return
        self.append_line(f">>> {cmd}")
        self.sig_send_command.emit(cmd)
        self._cmd_input.push_history(cmd)
        self._cmd_input.clear()
