"""Console widget for displaying raw serial output.

Simple append-only text viewer with line limit.
"""

from PySide6.QtWidgets import QWidget, QVBoxLayout, QPlainTextEdit
from PySide6.QtGui import QFont


class ConsoleWidget(QWidget):
    """Raw text log viewer with auto-scroll and line limit."""

    def __init__(self, max_lines: int = 5000, parent=None):
        super().__init__(parent)
        self._max_lines = max_lines

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

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
