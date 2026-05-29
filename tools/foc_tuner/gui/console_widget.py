"""Console widget for displaying raw serial output.

Simple append-only text viewer with line limit, clear, and save to file.
"""

from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QPushButton, QFileDialog
from PySide6.QtGui import QFont
from datetime import datetime


class ConsoleWidget(QWidget):
    """Raw text log viewer with auto-scroll and line limit."""

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
