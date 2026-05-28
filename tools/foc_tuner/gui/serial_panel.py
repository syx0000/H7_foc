"""Serial port connection panel.

Provides COM port selection, baud rate, connect/disconnect button,
and a separated Reset MCU button.
"""

from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QLabel, QComboBox, QPushButton, QMessageBox
)
from PySide6.QtCore import Signal


class SerialPanel(QWidget):
    """Serial port connection controls."""

    sig_connect_request = Signal(str, int)    # port_name, baud
    sig_disconnect_request = Signal()
    sig_reset_request = Signal()              # Reset MCU button

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False

        layout = QHBoxLayout(self)

        # Port selection
        layout.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(100)
        layout.addWidget(self._port_combo)

        # Refresh button
        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        layout.addWidget(self._refresh_btn)

        # Baud rate
        layout.addWidget(QLabel("Baud:"))
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(["921600", "115200", "57600", "9600"])
        self._baud_combo.setCurrentText("921600")
        layout.addWidget(self._baud_combo)

        # Connect button
        self._connect_btn = QPushButton("Connect")
        self._connect_btn.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self._connect_btn)

        # Spacer to keep Reset visually separated from connection controls
        layout.addSpacing(60)

        # Reset MCU button — disabled until connected
        self._reset_btn = QPushButton("Reset MCU")
        self._reset_btn.setStyleSheet("QPushButton { background-color: #ffe0e0; }")
        self._reset_btn.setEnabled(False)
        self._reset_btn.clicked.connect(self._on_reset_clicked)
        layout.addWidget(self._reset_btn)

        layout.addStretch()

        # Initial port scan
        self._refresh_ports()

    def _refresh_ports(self):
        """Scan and populate available COM ports."""
        from core.serial_worker import SerialWorker
        ports = SerialWorker.list_ports()
        self._port_combo.clear()
        self._port_combo.addItems(ports)

    def _on_connect_clicked(self):
        """Handle connect/disconnect button click."""
        if self._connected:
            self.sig_disconnect_request.emit()
        else:
            port = self._port_combo.currentText()
            baud = int(self._baud_combo.currentText())
            if port:
                self.sig_connect_request.emit(port, baud)

    def _on_reset_clicked(self):
        """Confirm and emit reset request."""
        reply = QMessageBox.question(
            self, "Reset MCU",
            "This will reboot the motor controller.\n"
            "Motor will stop, all RAM state lost. Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            self.sig_reset_request.emit()

    def set_connected(self, connected: bool):
        """Update UI state based on connection status.

        Args:
            connected: True if connected, False otherwise
        """
        self._connected = connected
        self._connect_btn.setText("Disconnect" if connected else "Connect")
        self._port_combo.setEnabled(not connected)
        self._baud_combo.setEnabled(not connected)
        self._refresh_btn.setEnabled(not connected)
        self._reset_btn.setEnabled(connected)
