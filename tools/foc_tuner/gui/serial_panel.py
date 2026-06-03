"""Serial / CAN-FD connection panel.

Provides backend selection (Serial / CAN), connection params,
connect/disconnect button, and a separated Reset MCU button.
"""

import json
import os
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QLabel, QComboBox, QPushButton, QMessageBox, QStackedWidget
)
from PySide6.QtCore import Signal


_CONNECTION_CONFIG_FILE = "foc_tuner_connection.json"


class SerialPanel(QWidget):
    """Connection controls (Serial or CAN-FD).

    sig_connect_request 第一个参数为 backend 字符串 ("serial" / "can"),
    后续参数视 backend 而定:
      - serial: (port_name:str, baud:int)
      - can:    (channel:int, abit:int, dbit:int)
    main_window 收到信号后用 backend 字段决定使用哪个 worker.
    """

    sig_connect_request = Signal(str, dict)   # backend, params
    sig_disconnect_request = Signal()
    sig_reset_request = Signal()              # Reset MCU button

    def __init__(self, parent=None):
        super().__init__(parent)
        self._connected = False

        layout = QHBoxLayout(self)

        # ---- Backend selection ----
        layout.addWidget(QLabel("Backend:"))
        self._backend_combo = QComboBox()
        self._backend_combo.addItems(["Serial", "CAN-FD"])
        self._backend_combo.currentIndexChanged.connect(self._on_backend_changed)
        layout.addWidget(self._backend_combo)

        # ---- StackedWidget: serial 参数 vs CAN 参数 ----
        self._stack = QStackedWidget()
        layout.addWidget(self._stack)

        # 串口参数页
        ser_w = QWidget()
        ser_l = QHBoxLayout(ser_w)
        ser_l.setContentsMargins(0, 0, 0, 0)
        ser_l.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(100)
        ser_l.addWidget(self._port_combo)
        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        ser_l.addWidget(self._refresh_btn)
        ser_l.addWidget(QLabel("Baud:"))
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(["921600", "115200", "57600", "9600"])
        self._baud_combo.setCurrentText("921600")
        ser_l.addWidget(self._baud_combo)
        self._stack.addWidget(ser_w)

        # CAN 参数页
        can_w = QWidget()
        can_l = QHBoxLayout(can_w)
        can_l.setContentsMargins(0, 0, 0, 0)
        can_l.addWidget(QLabel("CH:"))
        self._can_ch_combo = QComboBox()
        self._can_ch_combo.addItems(["0", "1"])
        can_l.addWidget(self._can_ch_combo)
        can_l.addWidget(QLabel("Arb:"))
        self._can_abit_combo = QComboBox()
        self._can_abit_combo.addItems(["1000000", "500000", "250000"])
        can_l.addWidget(self._can_abit_combo)
        can_l.addWidget(QLabel("Data:"))
        self._can_dbit_combo = QComboBox()
        self._can_dbit_combo.addItems(["5000000", "2000000", "1000000"])
        can_l.addWidget(self._can_dbit_combo)
        self._stack.addWidget(can_w)

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

        # 加载上次连接配置
        self._load_connection_config()

    def _load_connection_config(self):
        """加载上次的连接配置（backend / port / baud / can 参数）"""
        try:
            if not os.path.exists(_CONNECTION_CONFIG_FILE):
                return
            with open(_CONNECTION_CONFIG_FILE, 'r') as f:
                cfg = json.load(f)
            # Backend
            backend = cfg.get('backend', 'Serial')
            idx = 0 if backend == 'Serial' else 1
            self._backend_combo.setCurrentIndex(idx)
            # Serial 参数
            port = cfg.get('port', '')
            if port and self._port_combo.findText(port) >= 0:
                self._port_combo.setCurrentText(port)
            baud = cfg.get('baud', '921600')
            if self._baud_combo.findText(str(baud)) >= 0:
                self._baud_combo.setCurrentText(str(baud))
            # CAN 参数
            can_ch = cfg.get('can_channel', '0')
            if self._can_ch_combo.findText(str(can_ch)) >= 0:
                self._can_ch_combo.setCurrentText(str(can_ch))
            can_abit = cfg.get('can_abit', '1000000')
            if self._can_abit_combo.findText(str(can_abit)) >= 0:
                self._can_abit_combo.setCurrentText(str(can_abit))
            can_dbit = cfg.get('can_dbit', '5000000')
            if self._can_dbit_combo.findText(str(can_dbit)) >= 0:
                self._can_dbit_combo.setCurrentText(str(can_dbit))
        except Exception as e:
            print(f"[WARN] Failed to load connection config: {e}")

    def _save_connection_config(self):
        """保存当前连接配置"""
        try:
            cfg = {
                'backend': self._backend_combo.currentText(),
                'port': self._port_combo.currentText(),
                'baud': self._baud_combo.currentText(),
                'can_channel': self._can_ch_combo.currentText(),
                'can_abit': self._can_abit_combo.currentText(),
                'can_dbit': self._can_dbit_combo.currentText(),
            }
            with open(_CONNECTION_CONFIG_FILE, 'w') as f:
                json.dump(cfg, f, indent=2)
        except Exception as e:
            print(f"[WARN] Failed to save connection config: {e}")

    def _on_backend_changed(self, idx: int):
        self._stack.setCurrentIndex(idx)

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
            return
        backend = "serial" if self._backend_combo.currentIndex() == 0 else "can"
        # 保存连接配置（连接前保存，无论是否成功）
        self._save_connection_config()
        if backend == "serial":
            port = self._port_combo.currentText()
            baud = int(self._baud_combo.currentText())
            if port:
                self.sig_connect_request.emit("serial", {"port": port, "baud": baud})
        else:
            self.sig_connect_request.emit("can", {
                "channel": int(self._can_ch_combo.currentText()),
                "abit": int(self._can_abit_combo.currentText()),
                "dbit": int(self._can_dbit_combo.currentText()),
            })

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
        """Update UI state based on connection status."""
        self._connected = connected
        self._connect_btn.setText("Disconnect" if connected else "Connect")
        self._backend_combo.setEnabled(not connected)
        self._port_combo.setEnabled(not connected)
        self._baud_combo.setEnabled(not connected)
        self._refresh_btn.setEnabled(not connected)
        self._can_ch_combo.setEnabled(not connected)
        self._can_abit_combo.setEnabled(not connected)
        self._can_dbit_combo.setEnabled(not connected)
        self._reset_btn.setEnabled(connected)
